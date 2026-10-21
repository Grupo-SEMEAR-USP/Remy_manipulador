import math

import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2
from rclpy.callback_groups import ReentrantCallbackGroup
from manip_manipulation import kinematics
from geometry_msgs.msg import PointStamped
from manip_interfaces.msg import ManipStatus
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor


class MoveArm(Node):
    def __init__(self):
        super().__init__('move_arm')

        self.frame_braco = 'base_link'  # frame do braço, vamos ajustar depois
        # Tolerância lateral provisória para testes, em metros.
        self.y_tol = self.declare_parameter('y_tol', 0.02).value
        if not math.isfinite(self.y_tol) or self.y_tol < 0.0:
            raise ValueError('y_tol deve ser um número finito maior ou igual a zero.')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        callback_group = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=['junta_ombro', 'junta_cotovelo', 'junta_punho'],
            base_link_name='base_link',
            end_effector_name='flange_ferramenta',
            group_name='arm',
            callback_group=callback_group
        )
        
        self.moveit2.pipeline_id = 'ompl'

        self.status_pub = self.create_publisher(ManipStatus, '/manip/status', 10)

        self.sub = self.create_subscription(
            PointStamped,
            '/objeto_detectado',
            self.objeto_callback,
            10
        )

        self.get_logger().info('move_arm pronto, esperando coordenadas...')

    def publicar_status(self, estado, detalhe, erro_lateral=math.nan):
        status = ManipStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = self.frame_braco
        status.estado = estado
        status.erro_lateral = float(erro_lateral)
        status.tolerancia_lateral = float(self.y_tol)
        status.detalhe = detalhe
        self.status_pub.publish(status)

    def objeto_callback(self, msg: PointStamped):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_braco,
                msg.header.frame_id,
                msg.header.stamp,
                timeout=Duration(seconds=0.5)
            )
            ponto_convertido = do_transform_point(msg, transform)
        except Exception as e:
            self.get_logger().warn(f'Não consegui transformar: {e}')
            self.publicar_status(ManipStatus.FALHA, f'Não consegui transformar o alvo: {e}')
            return

        if not all(math.isfinite(v) for v in (
            ponto_convertido.point.x,
            ponto_convertido.point.y,
            ponto_convertido.point.z,
        )):
            detalhe = 'Alvo inválido: as coordenadas devem ser finitas.'
            self.get_logger().warn(detalhe)
            self.publicar_status(ManipStatus.FALHA, detalhe)
            return

        erro_lateral = ponto_convertido.point.y - kinematics.Y_PLANO
        if abs(erro_lateral) > self.y_tol:
            self.publicar_status(
                ManipStatus.PRECISA_ALINHAR,
                'Objeto fora do plano lateral do braço. Movimento não enviado.',
                erro_lateral,
            )
            self.get_logger().warn(
                'O robô ainda precisa se alinhar com o objeto. '
                f'No frame {self.frame_braco}: y={ponto_convertido.point.y:.3f} m, '
                f'plano do braço={kinematics.Y_PLANO:.3f} m, '
                f'erro lateral={erro_lateral:+.3f} m, '
                f'tolerância={self.y_tol:.3f} m. Movimento não enviado.'
            )
            return
        
        resultado = kinematics.solve_facing(
            ponto_convertido.point.x,
            ponto_convertido.point.z,
            standoff=0.10
        )

        if not resultado['ok']:
            self.get_logger().warn(f'Não alcancei o objeto: {resultado["motivo"]}')
            self.publicar_status(
                ManipStatus.FORA_DE_ALCANCE, resultado['motivo'], erro_lateral
            )
            return

        q1, q2, q3 = resultado['q']
        self.get_logger().info(
            f'Ângulos calculados: ombro={q1:.3f} cotovelo={q2:.3f} punho={q3:.3f}'
        )

        self.get_logger().info(
            f'Ponto no frame {self.frame_braco}: '
            f'x={ponto_convertido.point.x:.3f} '
            f'y={ponto_convertido.point.y:.3f} '
            f'z={ponto_convertido.point.z:.3f}'
        )
        try:
            enviado = self.moveit2.move_to_configuration([q1, q2, q3])
        except Exception as e:
            self.get_logger().error(f'Erro ao solicitar o movimento: {e}')
            self.publicar_status(ManipStatus.FALHA, f'Erro ao solicitar o movimento: {e}', erro_lateral)
            return
        if not enviado:
            motivo = self.moveit2.last_failure() or 'motivo não informado pelo MoveIt'
            self.get_logger().error(f'Pedido de movimento não enviado: {motivo}')
            self.publicar_status(ManipStatus.FALHA, f'Pedido não enviado: {motivo}', erro_lateral)
            return

        self.get_logger().info('Aguardando o planejamento e a execução pelo MoveIt...')
        self.publicar_status(
            ManipStatus.EM_MOVIMENTO,
            'Aguardando o planejamento e a execução pelo MoveIt.',
            erro_lateral,
        )
        try:
            sucesso = self.moveit2.wait_until_executed()
        except Exception as e:
            self.get_logger().error(f'Erro ao aguardar o movimento: {e}')
            self.publicar_status(ManipStatus.FALHA, f'Erro ao aguardar o movimento: {e}', erro_lateral)
            return
        if sucesso:
            self.get_logger().info('Movimento concluído com sucesso, confirmado pelo MoveIt.')
            self.publicar_status(
                ManipStatus.CONCLUIDO, 'Sucesso confirmado pelo MoveIt.', erro_lateral
            )
        else:
            motivo = self.moveit2.last_failure() or 'motivo não informado pelo MoveIt'
            self.get_logger().error(
                f'Movimento não concluído com sucesso: {motivo}'
            )
            self.publicar_status(ManipStatus.FALHA, motivo, erro_lateral)


def main():
    rclpy.init()
    node = MoveArm()
    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

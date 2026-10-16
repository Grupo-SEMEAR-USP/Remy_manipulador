import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2
from rclpy.callback_groups import ReentrantCallbackGroup
from manip_manipulation import kinematics
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor


class MoveArm(Node):
    def __init__(self):
        super().__init__('move_arm')

        self.frame_braco = 'base_link'  # frame do braço, vamos ajustar depois

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

        self.sub = self.create_subscription(
            PointStamped,
            '/objeto_detectado',
            self.objeto_callback,
            10
        )

        self.get_logger().info('move_arm pronto, esperando coordenadas...')

    def objeto_callback(self, msg: PointStamped):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_braco,
                msg.header.frame_id,
                msg.header.stamp,
                timeout=Duration(seconds=0.5)
            )
        except Exception as e:
            self.get_logger().warn(f'Não consegui transformar: {e}')
            return

        ponto_convertido = do_transform_point(msg, transform)
        
        resultado = kinematics.solve_facing(
            ponto_convertido.point.x,
            ponto_convertido.point.z,
            standoff=0.10
        )

        if not resultado['ok']:
            self.get_logger().warn(f'Não alcancei o objeto: {resultado["motivo"]}')
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
        self.moveit2.move_to_configuration([q1, q2, q3])
        self.moveit2.wait_until_executed()
        self.get_logger().info('Movimento concluído!')


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
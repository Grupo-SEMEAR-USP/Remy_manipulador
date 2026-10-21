# Teste integrado do manipulador no Gazebo

O launch `manip_gazebo.launch.py` inicia Gazebo, controladores, MoveIt, RViz,
`move_arm` e a transformação estática fictícia `base_link -> camera_link`.
Usa tempo simulado e não envia alvos automaticamente. A câmera fictícia é apenas
um frame coincidente com a base do braço; não produz imagens.

Encerre as instâncias individuais desses processos antes de usar este launch,
para evitar nós e transformações duplicados. Este teste mantém o braço isolado;
não integra a base móvel.

## Compilar no container

```bash
cd /remy_ws
colcon build --symlink-install --packages-up-to remy_bringup
source install/setup.bash
```

## Terminal 1: iniciar tudo

No computador:

```bash
docker exec -it remy_container bash
cd /remy_ws
source install/setup.bash
ros2 launch remy_bringup manip_gazebo.launch.py
```

Para outra tolerância, acrescente `y_tol:=0.01`, por exemplo. O padrão de
`0.02` m é provisório para simulação.

## Terminal 2: conferir e observar

No computador:

```bash
docker exec -it remy_container bash
cd /remy_ws
source install/setup.bash
ros2 control list_controllers
```

Aguarde `joint_state_broadcaster` e `arm_controller` ficarem `active`, a mensagem
`You can start planning now!` do MoveIt e o aviso de que `move_arm` está pronto.
Depois acompanhe os estados antes de publicar alvos:

```bash
ros2 topic echo /manip/status manip_interfaces/msg/ManipStatus
```

## Terminal 3: enviar alvos

No computador:

```bash
docker exec -it remy_container bash
cd /remy_ws
source install/setup.bash
```

Desalinhado: deve publicar `PRECISA_ALINHAR`, sem enviar movimento.

```bash
ros2 topic pub --once /objeto_detectado geometry_msgs/msg/PointStamped \
  "{header: {frame_id: camera_link}, point: {x: 0.55, y: 0.0, z: 0.45}}"
```

Alinhado e alcançável: deve publicar `EM_MOVIMENTO` e, se o MoveIt confirmar
sucesso, `CONCLUIDO`.

```bash
ros2 topic pub --once /objeto_detectado geometry_msgs/msg/PointStamped \
  "{header: {frame_id: camera_link}, point: {x: 0.55, y: -0.107782, z: 0.45}}"
```

Fora do alcance: deve publicar `FORA_DE_ALCANCE`, sem enviar movimento.

```bash
ros2 topic pub --once /objeto_detectado geometry_msgs/msg/PointStamped \
  "{header: {frame_id: camera_link}, point: {x: 2.0, y: -0.107782, z: 0.45}}"
```

Use Ctrl+C no terminal 1 para encerrar o conjunto.

# Estado do manipulador

O nó `move_arm` publica `manip_interfaces/msg/ManipStatus` em `/manip/status`.
As mensagens são eventos do processamento de alvos recebidos em
`/objeto_detectado`; não comandam a base móvel nem são publicadas periodicamente.
Abra o assinante antes de enviar o alvo para acompanhar todos os eventos.

| Estado | Significado |
| --- | --- |
| `PRECISA_ALINHAR` | Erro lateral acima da tolerância; movimento não enviado. |
| `FORA_DE_ALCANCE` | Sem solução de cinemática para o alvo alinhado lateralmente. |
| `EM_MOVIMENTO` | Pedido enviado; aguardando planejamento e execução. Não confirma movimento físico. |
| `CONCLUIDO` | Sucesso confirmado pelo MoveIt. |
| `FALHA` | Alvo inválido, falha de TF, envio, planejamento, execução ou cancelamento. Veja `detalhe`. |

`header.stamp` registra a publicação usando o relógio do nó;
`header.frame_id` identifica o frame do braço (`base_link` atualmente).
`erro_lateral` é `y_objeto - Y_PLANO`, em metros, após transformar o alvo para
esse frame. É `NaN` quando não há um alvo transformado válido.
`tolerancia_lateral` contém o parâmetro `y_tol`, em metros (padrão provisório:
`0.02`, para simulação). O sinal do erro não define sozinho um comando de
deslocamento da base: a navegação deve considerar os frames da montagem real.

Depois de alinhar a base, a integração deve fornecer uma nova medição do alvo.
O nó não retoma automaticamente um alvo rejeitado.

No container, compile e carregue:

```bash
cd /remy_ws
colcon build --symlink-install --packages-up-to manip_manipulation
source install/setup.bash
```

Em um terminal separado, dentro do container:

```bash
source /remy_ws/install/setup.bash
ros2 topic echo /manip/status manip_interfaces/msg/ManipStatus
```

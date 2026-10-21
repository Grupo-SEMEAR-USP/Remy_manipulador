# Estados referentes ao ultimo alvo processado; nao comandam a base movel.
string PRECISA_ALINHAR=PRECISA_ALINHAR
string FORA_DE_ALCANCE=FORA_DE_ALCANCE
string EM_MOVIMENTO=EM_MOVIMENTO
string CONCLUIDO=CONCLUIDO
string FALHA=FALHA

# Instante da publicacao; frame_id identifica o frame do braco.
std_msgs/Header header
string estado
# Metros: y do objeto no frame do braco menos Y_PLANO.
# NaN quando nao foi possivel obter uma coordenada lateral valida.
float64 erro_lateral
float64 tolerancia_lateral
string detalhe

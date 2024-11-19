import cv2
import time
import apriltag

'''# Parâmetros da conexão serial
SERIAL_PORT = '/dev/ttyS0'  # Altere para a porta correta
BAUD_RATE = 250000

# Inicia a comunicação serial
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Aguarda a inicialização da conexão
    print("Conexão serial estabelecida.")
except serial.SerialException as e:
    print(f"Falha ao estabelecer conexão serial: {e}")
    exit(1)'''

# Inicia a visão
camera = cv2.VideoCapture(3)
if not camera.isOpened():
    print("Erro ao capturar imagem.")
    exit(1)

# Ajusta a resolução
camera.set(3, 640)  # Largura
camera.set(4, 480)  # Altura

# Configuração para detector de AprilTags
detector = apriltag.Detector()

try:
    while True:
        # Captura um quadro
        status, frame = camera.read()
        if not status:
            print("Erro ao capturar imagem.")
            break

        # Converte o quadro para escala de cinza para a detecção de AprilTags
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = detector.detect(gray_frame)

        # Verifica se alguma AprilTag foi detectada
        if len(tags) > 0:
            print("AprilTag(s) detectada(s):")
            for tag in tags:
                tag_id = tag.tag_id
                print(f"ID da AprilTag detectada: {tag_id}")
                '''try:
                    # Envia o ID da tag para o Arduino como string, seguida por '\n' para indicar o fim da mensagem
                    ser.write(f'{tag_id}\n'.encode('utf-8'))
                    print(f"ID '{tag_id}' enviado para o Arduino.")
                except serial.SerialException:
                    print("Falha ao enviar o ID da tag para o Arduino.")'''

        # Ler resposta do Arduino, se houver
        '''if ser.in_waiting > 0:
            try:
                response = ser.readline().decode('utf-8').strip()
                print(f"Mensagem recebida do Arduino: {response}")
            except serial.SerialException:
                print("Erro ao ler resposta do Arduino.")'''

        # Pressione ESC para sair
        if cv2.waitKey(1) == 27:
            break

finally:
    # Fechar a conexão com a câmera e a comunicação serial
    camera.release()
    cv2.destroyAllWindows()
    '''ser.close()'''
    print("Recursos liberados com sucesso.")
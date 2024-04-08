import _thread
from socket import *


# 发送广播
def sendBroadcast():
    address = ('255.255.255.255', 10130)  # 255.255.255.255表示向任何网段发送广播消息 10130为端口号 发送端与接收端需一致
    s = socket(AF_INET, SOCK_DGRAM)  # 创建流式socket
    s.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
    message = b'This is broadcase message !'
    s.sendto(message, address)
    print(' send ok !')
    s.close()


# 接收广播
def receiveBroadcast():
    while True:
        address = ('', 10130)  # IP地址为空表示接收任何网段的广播消息
        s = socket(AF_INET, SOCK_DGRAM)
        s.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
        s.bind(address)
        print('Wait recv...')
        data, address = s.recvfrom(1024)
        print('[recv form %s:%d]:%s' % (address[0], address[1], data))
        s.close()


# 设置监听器，持续监听
def listenerBroadcast():
    print('Listener start...')
    _thread.start_new_thread(receiveBroadcast, ())  # 开启多线程持续监听


if __name__ == '__main__':
    listenerBroadcast()  # 接收端启动该函数
    sendBroadcast()  # 发送端启动该函数
#!/usr/bin/env pybricks-micropython

# Before running this program, make sure the client and server EV3 bricks are
# paired using Bluetooth, but do NOT connect them. The program will take care
# of establishing the connection.

# The server must be started before the client!

from pybricks.messaging import BluetoothMailboxServer, TextMailbox

def server_setup():
        
    server = BluetoothMailboxServer()
    mbox = TextMailbox('greeting', server)

    # The server must be started before the client!
    print('waiting for connection...')
    server.wait_for_connection()
    print('connected!')

    return mbox


def server_receive_hello(mbox):
    # In this program, the server waits for the client to send the first message
    # and then sends a reply.
    mbox.wait()
    print(mbox.read())
    mbox.send('hello to you!')


def server_receive(mbox):
    # Verifica se o cliente mandou mensagem, e retorna a propria mensagem
    if mbox.read():
        if mbox.read():
            message = mbox.read()
            print(message)
            return message
        else:
            return None


def server_send(mbox, message):
    mbox.send(message)
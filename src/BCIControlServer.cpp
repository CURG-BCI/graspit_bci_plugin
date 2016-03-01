//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// GraspIt! is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// GraspIt! is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with GraspIt!.  If not, see <http://www.gnu.org/licenses/>.
//
// Author(s): Andrew T. Miller
//
// $Id: graspitServer.cpp,v 1.7.4.1 2009/07/23 21:18:01 cmatei Exp $
//
//######################################################################

/*! \file
  \brief Implements the application's TCP server.
 */
#include <QDateTime>
#include <QTextStream>
#include <QApplication>
#include <iostream>
#include "BCIControlServer.h"
#include <sstream>

#include <QMutexLocker>
#include "BCI/bciService.h"
#include "include/debug.h"
#include <QDebug>


BCIControlServer::~BCIControlServer()
{
    std::cout << "Worker Destructor" << std::endl;
    delete server;
}

void
BCIControlServer::onConnection()
{
    socket = server->nextPendingConnection();


    if(socket->state() == QTcpSocket::ConnectedState)
     {
         printf("New connection established.\n");
         qDebug()<<socket->peerAddress();
     }
     connect(socket, SIGNAL(disconnected()),
     this, SLOT(on_disconnected()));
     connect(socket, SIGNAL(readyRead()),
     this, SLOT(on_readyRead()));

}

void BCIControlServer::on_readyRead()
{
    while(socket->canReadLine())
    {

        QString state_string=socket->readLine();
       int state= state_string.toInt();
       if(state==1)
       {
           BCIService::getInstance()->emitGoToStateLow();
       }
       else if(state==2)
       {
           BCIService::getInstance()->emitGoToStateMedium();
       }
       else if(state==3)
       {
           BCIService::getInstance()->emitGoToStateHigh();
       }

    }
}

void BCIControlServer::on_disconnected()
{
    printf("Connection disconnected.\n");
    disconnect(socket, SIGNAL(disconnected()));
    disconnect(socket, SIGNAL(readyRead()));
    socket->deleteLater();
}


void BCIControlServer::process() {
    int port_num = 4775;
    server = new QTcpServer(this);
    connect(server, SIGNAL(newConnection()), this, SLOT(onConnection()));
    server->listen(QHostAddress::LocalHost,port_num);
}

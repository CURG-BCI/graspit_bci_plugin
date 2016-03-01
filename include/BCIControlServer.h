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
// Author(s):  Andrew T. Miller
//
// $Id: graspitServer.h,v 1.3.4.1 2009/07/23 21:17:43 cmatei Exp $
//
//######################################################################

/*! \file
  \brief Defines the GraspItServer and the ClientSocket classes
 */

#ifndef GRASPITSERVER_HXX
#include <QTcpSocket>
#include <QTcpServer>
#include <qstringlist.h>
#include <iostream>
#include <vector>
#include <QThread>


class BCIControlServer : public QObject {
    Q_OBJECT
    QTcpSocket *socket;
    QTcpServer* server;

public:
    ~BCIControlServer();

public slots:
    void process();
    void onConnection();
    void on_disconnected();
    void on_readyRead();

signals:
    void finished();
    void error(QString err);

};


#define GRASPITSERVER_HXX
#endif

/*
 * esmini - Environment Simulator Minimalistic
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

#include <stdio.h>

#ifndef _WIN32
#include <sys/time.h>
#endif

#include "UDP.hpp"
#include "CommonMini.hpp"

UDPBase::UDPBase(unsigned short int port) : port_(port), sock_(SE_INVALID_SOCKET)
{
    sender_addr_size_ = sizeof(sender_addr_);

#ifdef _WIN32
    WSADATA wsa_data;
    int     iResult = WSAStartup(MAKEWORD(2, 2), &wsa_data);
    if (iResult != NO_ERROR)
    {
        wprintf(L"WSAStartup failed with error %d\n", iResult);
        return;
    }
#endif

    if ((sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SE_INVALID_SOCKET)
    {
        LOG_AND_QUIT("socket failed");
        return;
    }
}

int UDPBase::Bind(struct sockaddr_in& addr)
{
    int retval = bind(sock_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr));
    if (retval != 0)
    {
#ifdef _WIN32
        LOG("bind error %d", WSAGetLastError());
#else
        perror("bind socket");
        LOG("bind socket error");
#endif
        CloseGracefully();
        LOG_AND_QUIT("Bind UDP socket on port %d failed (return code %d)", port_, retval);
        return -1;
    }

    return 0;
}

void UDPBase::CloseGracefully()
{
#ifdef _WIN32
    if (closesocket(sock_) == SOCKET_ERROR)
#else
    if (close(sock_) < 0)
#endif
    {
#ifdef _WIN32
        LOG("Failed closing socket %d", WSAGetLastError());
#else
        perror("close socket");
        LOG("close socket error");
#endif
    }

#ifdef _WIN32
    WSACleanup();
#endif
}

UDPServer::UDPServer(unsigned short int port, unsigned int timeoutMs) : UDPBase(port), timeoutMs_(timeoutMs)
{
    // set timer for receive operations
#ifdef _WIN32
    if (setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeoutMs_, sizeof(timeoutMs_)) != 0)
#else
    struct timeval tv;
    tv.tv_sec = timeoutMs_ / 1000;
    tv.tv_usec = (timeoutMs_ % 1000) * 1000;
    if (setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
#endif
    {
        printf("socket SO_RCVTIMEO (receive timeout) not supported on this platform\n");
    }

    // Prepare the sockaddr_in structure
    server_addr_.sin_family      = AF_INET;
    server_addr_.sin_port        = htons(port_);
    server_addr_.sin_addr.s_addr = htonl(INADDR_ANY);

    Bind(server_addr_);
}

int UDPServer::Receive(char* buf, unsigned int size)
{
    // TODO:
    // Casting to int can cause overflow in this situation. Not a good idea.
    // Let's fix it in a way that we actually return size_t and design the flow like that
    return static_cast<int>(recvfrom(sock_, buf, size, 0, reinterpret_cast<struct sockaddr*>(&sender_addr_), &sender_addr_size_));
}

UDPClient::UDPClient(unsigned short int port, std::string ipAddress) : UDPBase(port), ipAddress_(ipAddress)
{
    // Prepare the sockaddr_in structure
    memset(reinterpret_cast<char*>(&server_addr_), 0, sizeof(server_addr_));
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port   = htons(port_);
    inet_pton(AF_INET, ipAddress.c_str(), &server_addr_.sin_addr.s_addr);
}

int UDPClient::Send(char* buf, unsigned int size)
{
    // TODO:
    // Casting to int can cause overflow in this situation. Not a good idea.
    // Let's fix it in a way that we actually return size_t and design the flow like that
    return static_cast<int>(sendto(sock_, buf, size, 0, reinterpret_cast<struct sockaddr*>(&server_addr_), sizeof(server_addr_)));
}
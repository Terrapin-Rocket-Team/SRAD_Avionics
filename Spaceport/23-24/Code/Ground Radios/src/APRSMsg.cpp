/*
MIT License

Copyright (c) 2020 Peter Buchegger

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "APRSMsg.h"

APRSMsg::APRSMsg() : _body()
{
}

APRSMsg::APRSMsg(APRSMsg &other_msg)
    : _type(other_msg.getType()), _body()
{
    strcpy(_source, other_msg.getSource());
    strcpy(_destination, other_msg.getDestination());
    strcpy(_path, other_msg.getPath());
    strcpy(_rawBody, other_msg.getRawBody());
    _body.setData(other_msg.getBody()->getData());
}

APRSMsg &APRSMsg::operator=(APRSMsg &other_msg)
{
    if (this != &other_msg)
    {
        setSource(other_msg.getSource());
        setDestination(other_msg.getDestination());
        setPath(other_msg.getPath());
        _type = other_msg.getType();
        strcpy(_rawBody, other_msg.getRawBody());
        _body.setData(other_msg.getBody()->getData());
    }
    return *this;
}

APRSMsg::~APRSMsg()
{
}

const char *APRSMsg::getSource()
{
    return _source;
}

void APRSMsg::setSource(const char source[8])
{
    strcpy(_source, source);
}

const char *APRSMsg::getDestination()
{
    return _destination;
}

void APRSMsg::setDestination(const char destination[8])
{
    strcpy(_destination, destination);
}

const char *APRSMsg::getPath()
{
    return _path;
}

void APRSMsg::setPath(const char path[10])
{
    strcpy(_path, path);
}

APRSMessageType APRSMsg::getType()
{
    return _type;
}

const char *APRSMsg::getRawBody()
{
    return _rawBody;
}

APRSBody *APRSMsg::getBody()
{
    return &_body;
}

bool APRSMsg::decode(char *message)
{
    int len = strlen(message);
    int pos_src = -1, pos_dest = -1, pos_path = -1;
    for (int i = 0; i < len; i++)
    {
        if (message[i] == '>' && pos_src == -1)
            pos_src = i;
        if (message[i] == ',' && pos_dest == -1)
            pos_dest = i;
        if (message[i] == ':' && pos_path == -1)
            pos_path = i;
    }
    if (pos_src >= 8)
        return false;
    if (pos_dest - (pos_src + 1) >= 8)
        return false;
    if (pos_path - (pos_dest + 1) >= 10)
        return false;

    if (pos_src >= 0)
    {
        strncpy(_source, message, pos_src);
        _source[pos_src] = '\0';
    }
    else
    {
        _source[0] = '\0';
    }

    if (pos_dest != -1 && pos_dest < pos_path)
    {
        strncpy(_path, message + pos_dest + 1, pos_path - (pos_dest + 1));
        strncpy(_destination, message + pos_src + 1, pos_dest - (pos_src + 1));
        _path[pos_path - (pos_dest + 1)] = '\0';
        _destination[pos_dest - (pos_src + 1)] = '\0';
    }
    else
    {
        _path[0] = '\0';
        if (pos_src >= 0 && pos_path >= 0)
        {

            strncpy(_destination, message + pos_src + 1, pos_path - (pos_src + 1));
            _destination[pos_path - (pos_src + 1)] = '\0';
        }
        else
        {
            _destination[0] = '\0';
        }
    }
    strcpy(_rawBody, message + pos_path + 1);
    _rawBody[strlen(message + pos_path + 1)] = '\0';
    _type = APRSMessageType(_rawBody[0]);
    _body.decode(_rawBody);
    return bool(_type);
}

void APRSMsg::encode(char *message)
{
    sprintf(message, "%s>%s", _source, _destination);
    if (strlen(_path) > 0)
    {
        sprintf(message + strlen(message), ",%s", _path);
    }
    sprintf(message + strlen(message), ":%s", _body.encode());
}

void APRSMsg::toString(char *str)
{
    char body[87];
    _body.toString(body);
    sprintf(str, "Source:%s,Destination:%s,Path:%s,Type:%s,%s", _source, _destination, _path, _type.toString(), body);
}

APRSBody::APRSBody()
{
}

APRSBody::~APRSBody()
{
}

const char *APRSBody::getData()
{
    return _data;
}

void APRSBody::setData(const char data[80])
{
    strcpy(_data, data);
}

bool APRSBody::decode(char *message)
{
    strcpy(_data, message);
    return true;
}

const char *APRSBody::encode()
{
    return _data;
}

void APRSBody::toString(char *str)
{
    sprintf(str, "Data:%s", _data);
}
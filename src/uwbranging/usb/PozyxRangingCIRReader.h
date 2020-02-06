/*
MIT License

Copyright (c) 2020 Group of Electronic Technology and Communications. University of A Coruna.

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

#ifndef POZYX_RANGING_CIR_READER_H
#define POZYX_RANGING_CIR_READER_H

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/assert.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>

#include <string>
#include <vector>
#include <stdint.h>
#include <stdio.h>
#include <iostream>
#include <math.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int16MultiArray.h"
#include <sstream>
#include <gtec_msgs/PozyxRangingWithCir.h>

typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;
#define SERIAL_PORT_READ_BUF_SIZE 5000
#define TOF_REPORT_LEN  4088

class DataAnchor;
class DataTag;

#define PORT_TYPE_UWB 0


#define DWT_PRF_16M     1   //!< UWB PRF 16 MHz
#define DWT_PRF_64M     2   //!< UWB PRF 64 MHz

#define DWT_BR_110K     0   //!< UWB bit rate 110 kbits/s
#define DWT_BR_850K     1   //!< UWB bit rate 850 kbits/s
#define DWT_BR_6M8      2   //!< UWB bit rate 6.8 Mbits/s

#define DWT_PLEN_4096   0x0C    //! Standard preamble length 4096 symbols
#define DWT_PLEN_2048   0x28    //! Non-standard preamble length 2048 symbols
#define DWT_PLEN_1536   0x18    //! Non-standard preamble length 1536 symbols
#define DWT_PLEN_1024   0x08    //! Standard preamble length 1024 symbols
#define DWT_PLEN_512    0x34    //! Non-standard preamble length 512 symbols
#define DWT_PLEN_256    0x24    //! Non-standard preamble length 256 symbols
#define DWT_PLEN_128    0x14    //! Non-standard preamble length 128 symbols
#define DWT_PLEN_64     0x04    //! Standard preamble length 64 symbols

#define MAX_NUM_TAGS (8)
#define MAX_NUM_ANCS (4)


typedef struct
{
    double x, y, z;
    uint64_t id;
} pos_report_t;

typedef struct
{
    double x;
    double y;
} vec2d;

class PozyxRangingCIRReader
{

public:
     PozyxRangingCIRReader();
    ~PozyxRangingCIRReader();

    int openSerialPort(std::string name,int portType); //open selected serial port
    void start(std::string usbPort, ros::Publisher aPub);
    void newData(const std::vector<char> data);

protected:
    boost::mutex mutex_;
    char end_of_line_char_;
    char read_buf_raw_[SERIAL_PORT_READ_BUF_SIZE];
    std::vector<char> buff_to_process;
    std::string read_buf_str_;
    boost::asio::io_service io_service_;
    virtual void async_read_some_();
    virtual void on_receive_(const boost::system::error_code& ec, size_t bytes_transferred);
private:

    serial_port_ptr _serialUWB;
    ros::Publisher ros_pub;
    std::string uwb_port_name;
    boost::thread *thread_uwb;
    bool _header_loaded;
    double _lastAngle;
};

#endif //POZYX_RANGING_CIR_READER_H
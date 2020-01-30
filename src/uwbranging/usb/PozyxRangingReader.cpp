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

#include "PozyxRangingReader.h"


#define INST_REPORT_LEN   (20)
#define INST_VERSION_LEN  (16)
#define INST_CONFIG_LEN   (1)


PozyxRangingReader::PozyxRangingReader() {
    _header_loaded = false;
}

PozyxRangingReader::~PozyxRangingReader() {
    if (_serialUWB) {
        _serialUWB->close();
    }
}

void PozyxRangingReader::newAngle(const std_msgs::Float64 newAngle){

    _lastAngle = newAngle.data;
}

int PozyxRangingReader::openSerialPort(std::string name, int portType) {
    int error = -1;
    boost::system::error_code errorCode;

    if (portType == PORT_TYPE_UWB) {
        if (!_serialUWB) {
            error = 0;
            try {

                _serialUWB = serial_port_ptr(new boost::asio::serial_port(io_service_));
                _serialUWB->open(name, errorCode);
                if (errorCode.value() != 0) {
                    std::cout << "Error code opening port: " << errorCode.message() << "\n";
                    return (-1);
                }

                _serialUWB->set_option(boost::asio::serial_port_base::baud_rate(115200));
                _serialUWB->set_option(boost::asio::serial_port_base::character_size(8));
                _serialUWB->set_option(
                    boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
                _serialUWB->set_option(
                    boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
                _serialUWB->set_option(
                    boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

                //std::string decaIni = "deca$";
                //size_t written = writeData(decaIni.c_str(), decaIni.length(), PORT_TYPE_UWB);

            } catch (boost::system::system_error boostError) {
                error = -1;
            }
        }
    }

    return error;
}


// int PozyxRangingReader::writeData(const char *data, int size, int portType) {

//     boost::system::error_code ec;
//     if (size == 0) return 0;
//     if (portType == PORT_TYPE_UWB) {
//         if (_serialUWB) {
//             size_t written = _serialUWB->write_some(boost::asio::buffer(data, size), ec);
//             if (ec.value() == 0) {
//                 return written;
//             } else {
//                 std::cout << "Write error: " << ec.message() << "\n";
//                 return -1;
//             }

//         } else {
//             return -1;
//         }
//     }
//     return -1;
// }


// void PozyxRangingReader::newHeaderData(const std::string &data) {

//     int length = data.length();
//     int offset = 0;
//     if (length >= INST_REPORT_LEN) {
//         while (length >= INST_REPORT_LEN) {
//             std::string header = data.substr(offset, 2);
//             std::string headerValue("nV");
//             if (header.compare(0, headerValue.length(), headerValue) == 0) {
//                 _header_loaded = true;
//                 break;
//             }

//             offset += 2;
//             length -= 2;
//         }

//         if (length < INST_REPORT_LEN) {
//             return;
//         }
//     }
// }


void PozyxRangingReader::async_read_some_() {

    //ROS_INFO("async_read_some");

    if (_serialUWB.get() == NULL || !_serialUWB->is_open()) {
        ROS_INFO("DEBUG:SeriaUSB ==NULL || !is_open()");
        return;
    }

    _serialUWB->async_read_some(
        boost::asio::buffer(read_buf_raw_, SERIAL_PORT_READ_BUF_SIZE),
        boost::bind(
            &PozyxRangingReader::on_receive_,
            this, boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
}

void PozyxRangingReader::on_receive_(const boost::system::error_code &ec, size_t bytes_transferred) {

    bool hasError = false;

    //Deberia ser un mensaje de ranging de UWB
    if (_serialUWB.get() == NULL || !_serialUWB->is_open()) {
        ROS_INFO("DEBUG:SeriaUSB ==NULL || !is_open()");
        hasError = true;
    }

    if (ec) {
        ROS_INFO("DEBUG:Error Code %d, %s ", (int) ec.value(), ec.message().c_str());
        hasError = true;
    }


    if (!hasError) {
        for (unsigned int i = 0; i < bytes_transferred; ++i) {
            char c = read_buf_raw_[i];
            buff_to_process.push_back(c);
        }

        //ROS_INFO("DEBUG:buff_to_process size %d", (int) buff_to_process.size());
        if (buff_to_process.size() >= TOF_REPORT_LEN) {
            bool somethingToProcess = true;
            while (somethingToProcess) {
                somethingToProcess = false;
                int headerPos = 0;
                bool headerFound = false;

                int bufLength = (int) buff_to_process.size();

                while (headerPos < bufLength-1) {

                    if (((uint8_t)buff_to_process[headerPos] == 0xFA) && ((uint8_t)buff_to_process[headerPos + 1] == 0xFA)) {
                        headerFound = true;
                        //ROS_INFO("DEBUG:Header found");
                        break;
                    }
                    headerPos += 1;
                }

                if (!headerFound) {
                    //ROS_INFO("DEBUG:Header NOT found");
                    //Borramos todo el buffer pendiente y salimos, ya que en el buffer no hay ningun
                    //mensaje completo
                    //Podria quedar solo el primer byte de una cabecera, lo comprobamos
                    if ((uint8_t) buff_to_process[bufLength-1] == 0xFA){
                        buff_to_process.erase(buff_to_process.begin(), buff_to_process.end() - 1);
                    } else {
                        //No hay nada util, borramos todo
                        buff_to_process.clear();
                    }
                    break;
                } else {
                   // ROS_INFO("DEBUG:Header found HeaderPos: %d", headerPos);
                    //Comprobamos si en el buffer queda suficiente espacio para un mensaje completo
                    bool enoughSize = (headerPos + TOF_REPORT_LEN) <= bufLength;

                    if (enoughSize) {
                       // ROS_INFO("DEBUG:has enoughSize");
                        //Comprobamos si la cola es correcta
                        bool tailFound = false;
                       // ROS_INFO("DEBUG:Tail pos byte: %x", (uint8_t) buff_to_process[headerPos + TOF_REPORT_LEN - 2]);
                        if (((uint8_t) buff_to_process[headerPos + TOF_REPORT_LEN - 2] == 0xBB) && ((uint8_t) buff_to_process[headerPos + TOF_REPORT_LEN - 1] == 0xBB)) {
                            tailFound = true;
                             //ROS_INFO("DEBUG:TAIL FOUND ++++");
                        }

                        if (tailFound) {
                            //Decodificamos
                            std::vector<char>::const_iterator first = buff_to_process.begin() + headerPos;
                            std::vector<char>::const_iterator last = buff_to_process.begin() + headerPos + TOF_REPORT_LEN;
                            std::vector<char> newRanging(first, last);
                            newData(newRanging);

                            //Borramos todos los datos incluida la ultima cola
                            buff_to_process.erase(buff_to_process.begin(), buff_to_process.begin() + headerPos + TOF_REPORT_LEN);

                        } else {
                            //No hay cola, estamos seguros que desde header_pos a header_pos + TOF_REPORT_LEN no hay mensaje
                            //Borramos los datos en esa parte
                            buff_to_process.erase(buff_to_process.begin(), buff_to_process.begin() + headerPos + TOF_REPORT_LEN);
                        }

                        //Comprobamos si sigue quedando espacio para almacenar un mensaje
                        if (buff_to_process.size() >= TOF_REPORT_LEN) {
                            somethingToProcess = true;
                        }

                    } else {
                         //ROS_INFO("DEBUG:has NOT enoughSize");
                        //Se trata de un mensaje sin cola, tenemos que esperar que haya mas bytes
                        //No hacemos nada y ya se sale
                        break;
                    }
                }
            }
        }

        async_read_some_();
    }

}




void PozyxRangingReader::start(std::string usbPort, ros::Publisher aPub) {

    ROS_INFO("Iniciando UWB receiver");
    uwb_port_name = usbPort;
    ros_pub = aPub;

    //Abrimos el puerto por donde recibiremos las medidas de UWB
    ROS_INFO("Intentando abrir puerto %s", uwb_port_name.c_str());
    bool uwbPortOpen = (openSerialPort(uwb_port_name, PORT_TYPE_UWB) == 0);

    if (uwbPortOpen) {
        ROS_INFO("Puerto abierto");

        //Empezamos a leer asincronamente desde el puerto UWB y cuando recibamos datos los parseamos,
        async_read_some_();
        boost::thread t(boost::bind(&boost::asio::io_service::run, &io_service_));

    } else {
        ROS_INFO("No se pudo abrir el puerto");
    }

}


void PozyxRangingReader::newData(const std::vector<char> data) {

    //ROS_INFO("DEBUG:NEW DATA ****");

    std::string nowstr = "T:hhmmsszzz:";
    int originType,destinationType, range, rangetime, seq, prf, channel, datarate, prfValue;
    double channelValue, datarateValue;

    int16_t originId, destinationId;
    int16_t rss;
    double angle = _lastAngle;

    originId = int((uint8_t)(data[3]) << 8 |
                    (uint8_t)(data[2]));

    originType = data[4];
    destinationId = int((uint8_t)(data[6]) << 8 |
                    (uint8_t)(data[5]));
    destinationType = data[7];

    range = int((uint8_t)(data[11]) << 24 |
                (uint8_t)(data[10]) << 16 |
                (uint8_t)(data[9]) << 8 |
                (uint8_t)(data[8]));

    rangetime = int((uint8_t)(data[15]) << 24 |
                    (uint8_t)(data[14]) << 16 |
                    (uint8_t)(data[13]) << 8 |
                    (uint8_t)(data[12]));

    //ROS_INFO("DEBUG:seq pos byte: %x",  (uint8_t)data[14]);
    seq = (uint8_t)data[16];
    
    rss = int((uint8_t)(data[18]) << 8 |
                    (uint8_t)(data[17]));

    channel = (uint8_t)data[19];
    datarate = (uint8_t)data[20];
    prf = (uint8_t)data[21];


        if (channel == 1) {
            channelValue = 3494.4;
        } else if (channel == 2) {
            channelValue = 3993.6;
        } else if (channel == 3) {
            channelValue = 4492.8;
        } else if (channel == 4) {
            channelValue = 3993.6;
        } else if (channel == 5) {
            channelValue = 6489.6;
        } else if (channel == 7) {
            channelValue = 6489.6;
        } else {
            channelValue = -1.0;
        }

        if (prf == DWT_PRF_16M) {
            prfValue = 16;
        } else if (prf == DWT_PRF_64M) {
            prfValue = 64;
        } else {
            prfValue = -1;
        }

        if (datarate == DWT_BR_110K) {
            datarateValue = 0.11;
        } else if (datarate == DWT_BR_6M8) {
            datarateValue = 6.8;
        } else if (datarate == DWT_BR_850K) {
            datarateValue = 0.85;
        } else {
            datarateValue = -1.0;
        }


    gtec_msgs::PozyxRanging ranging_msg;

    ranging_msg.originId = originId;
    ranging_msg.originType = originType;
    ranging_msg.destinationId = destinationId;
    ranging_msg.destinationType = destinationType;
    ranging_msg.range = range;
    ranging_msg.seq = seq;
    ranging_msg.ts = rangetime;
    ranging_msg.rxPower = rss;
    ranging_msg.channel = channelValue;
    ranging_msg.prf = prfValue;
    ranging_msg.datarate = datarateValue;
    ranging_msg.preambleLength = 0;
    ranging_msg.txGain = 0;
    ranging_msg.angle = angle;

    if (seq <= 255) {
        ros_pub.publish(ranging_msg);
    }

}


// void PozyxRangingReader::newData(const std::string &data) {

//     std::string nowstr = "T:hhmmsszzz:";
//     int length = data.length();
//     int idx = 0, lastSeq = 0, count = 0;
//     bool trilaterate = false;
//     bool newposition = false;
//     int offset = 0;
//     int nolocation = 0;
//     std::string statusMsg;


//     while (length >= TOF_REPORT_LEN) //we have received a range report from an anchor or a tag
//     {
//         while (length >= TOF_REPORT_LEN) {

//             std::string header = data.substr(offset, 2);

//             if (header.compare("ma") == 0) //loop here until we reach header ("ma")
//             {
//                 break;
//             }

//             offset += 2;
//             length -= 2;
//         }

//         if (length < TOF_REPORT_LEN) {
//             return;
//         }


//         std::string tofReport = data.substr(offset, TOF_REPORT_LEN);


//         //POZYX version
//         // ido - identificador nodo origen
//         // ot - tipo nodo origen: 0: tag, 1: anchor
//         // idd - identificador nodo destino
//         // dt - tipo nodo destino: 0: tag, 1:anchor
//         // r - ranging (mm)
//         // ts - timestamp (ms)
//         // tx - RSS (dBm)
//         // sq - seq number
//         // ch - UWB channel: 1, 2, 3, 4, 5, 7
//         // dr - Data rate: 0: 110kbits/s, 1: 850kbits/s, 2: 6.8Mbits/s.
//         // pr - prf: 1: 16MHz, 2: 64MHz
//         // pl - preamble length 0x0C : 4096 symbols., 0x28 : 2048 symbols., 0x18 : 1536 symbols., 0x08 : 1024 symbols., 0x34 : 512 symbols., 0x24 : 256 symbols., 0x14 : 128 symbols., 0x04 : 64 symbols.
//         // g - transmision gain 0dB - 33.5 db, steps 0.5
//         //ma ido6044 ot0 idd6040 dt1 r000005d4 ts000005d4 tx00AA sq01 ch00 dr00 pr00 pl00 g00


//         int originId, originType, destinationId, destinationType, range, rangetime, seq, rss;
//         int channel, prf, datarate, preambleLength, gain;

//         //                                 ma ido0 ot0 idd3 dt1 r000017E8 ts0000A497 txFFA5 sqE9 ch05 dr00 pr02 pl08 g16
//         int n = sscanf(tofReport.c_str(), "ma ido%x ot%x idd%x dt%x r%x ts%x tx%x sq%x ch%x dr%x pr%x pl%x g%x", &originId, &originType, &destinationId, &destinationType, &range, &rangetime, &rss, &seq, &channel, &datarate, &prf, &preambleLength, &gain);

//         double channelValue, datarateValue;
//         int prfValue, preambleLengthValue, pacSizeValue;

//         if (channel == 1) {
//             channelValue = 3494.4;
//         } else if (channel == 2) {
//             channelValue = 3993.6;
//         } else if (channel == 3) {
//             channelValue = 4492.8;
//         } else if (channel == 4) {
//             channelValue = 3993.6;
//         } else if (channel == 5) {
//             channelValue = 6489.6;
//         } else if (channel == 7) {
//             channelValue = 6489.6;
//         } else {
//             channelValue = -1.0;
//         }

//         if (prf == DWT_PRF_16M) {
//             prfValue = 16;
//         } else if (prf == DWT_PRF_64M) {
//             prfValue = 64;
//         } else {
//             prfValue = -1;
//         }

//         if (datarate == DWT_BR_110K) {
//             datarateValue = 0.11;
//         } else if (datarate == DWT_BR_6M8) {
//             datarateValue = 6.8;
//         } else if (datarate == DWT_BR_850K) {
//             datarateValue = 0.85;
//         } else {
//             datarateValue = -1.0;
//         }

//         switch (preambleLength) {
//         case DWT_PLEN_4096:
//             preambleLengthValue = 4096;
//             break;
//         case DWT_PLEN_2048:
//             preambleLengthValue = 2048;
//             break;
//         case DWT_PLEN_1536:
//             preambleLengthValue = 1536;
//             break;
//         case DWT_PLEN_1024:
//             preambleLengthValue = 1024;
//             break;
//         case DWT_PLEN_512:
//             preambleLengthValue = 512;
//             break;
//         case DWT_PLEN_256:
//             preambleLengthValue = 256;
//             break;
//         case DWT_PLEN_128:
//             preambleLengthValue = 128;
//             break;
//         case DWT_PLEN_64:
//             preambleLengthValue = 64;
//             break;
//         default:
//             preambleLengthValue = -1;
//             break;
//         }


//         offset += TOF_REPORT_LEN;
//         length -= TOF_REPORT_LEN;

//         gtec_msgs::PozyxRanging ranging_msg;

//         ranging_msg.originId = originId;
//         ranging_msg.originType = originType;
//         ranging_msg.destinationId = destinationId;
//         ranging_msg.destinationType = destinationType;
//         ranging_msg.range = range;
//         ranging_msg.seq = seq;
//         ranging_msg.ts = rangetime;
//         ranging_msg.rxPower = rss;
//         ranging_msg.channel = channelValue;
//         ranging_msg.prf = prfValue;
//         ranging_msg.datarate = datarateValue;
//         ranging_msg.preambleLength = preambleLengthValue;
//         ranging_msg.txGain = gain;

//         if (seq > 255) {
//             //ERROR
//             //ROS_INFO("ERROR: MENSAJE TRUNCADO RR:[OriginId:%x, Destination:%x, Range:%d, SEQ:%d]", originId, destinationId, range, seq);
//             ROS_INFO("DEBUG:LINEA  {{{[ %s ]}}}", data.c_str());
//         } else {
//             //Publish array
//             ros_pub.publish(ranging_msg);
//         }
//     }
//}

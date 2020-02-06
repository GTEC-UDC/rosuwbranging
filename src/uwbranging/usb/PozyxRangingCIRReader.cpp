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

#include "PozyxRangingCIRReader.h"


#define INST_REPORT_LEN   (20)
#define INST_VERSION_LEN  (16)
#define INST_CONFIG_LEN   (1)


PozyxRangingCIRReader::PozyxRangingCIRReader() {
    _header_loaded = false;
}

PozyxRangingCIRReader::~PozyxRangingCIRReader() {
    if (_serialUWB) {
        _serialUWB->close();
    }
}


int PozyxRangingCIRReader::openSerialPort(std::string name, int portType) {
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

void PozyxRangingCIRReader::async_read_some_() {

    //ROS_DEBUG("async_read_some");

    if (_serialUWB.get() == NULL || !_serialUWB->is_open()) {
        ROS_DEBUG("DEBUG:SeriaUSB ==NULL || !is_open()");
        return;
    }

    _serialUWB->async_read_some(
        boost::asio::buffer(read_buf_raw_, SERIAL_PORT_READ_BUF_SIZE),
        boost::bind(
            &PozyxRangingCIRReader::on_receive_,
            this, boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
}

void PozyxRangingCIRReader::on_receive_(const boost::system::error_code &ec, size_t bytes_transferred) {

    bool hasError = false;

    //Deberia ser un mensaje de ranging de UWB
    if (_serialUWB.get() == NULL || !_serialUWB->is_open()) {
        ROS_DEBUG("DEBUG:SeriaUSB ==NULL || !is_open()");
        hasError = true;
    }

    if (ec) {
        ROS_DEBUG("DEBUG:Error Code %d, %s ", (int) ec.value(), ec.message().c_str());
        hasError = true;
    }


    if (!hasError) {
        for (unsigned int i = 0; i < bytes_transferred; ++i) {
            char c = read_buf_raw_[i];
            buff_to_process.push_back(c);
        }

        ROS_DEBUG("DEBUG:buff_to_process size %d", (int) buff_to_process.size());
        if (buff_to_process.size() >= TOF_REPORT_LEN) {
            bool somethingToProcess = true;
            while (somethingToProcess) {
                somethingToProcess = false;
                int headerPos = 0;
                bool headerFound = false;

                int bufLength = (int) buff_to_process.size();

                while (headerPos < bufLength - 1) {

                    if (((uint8_t)buff_to_process[headerPos] == 0xFA) && ((uint8_t)buff_to_process[headerPos + 1] == 0xFA)) {
                        headerFound = true;
                        ROS_DEBUG("DEBUG:Header found");
                        break;
                    }
                    headerPos += 1;
                }

                if (!headerFound) {
                    ROS_DEBUG("DEBUG:Header NOT found");
                    //Borramos todo el buffer pendiente y salimos, ya que en el buffer no hay ningun
                    //mensaje completo
                    //Podria quedar solo el primer byte de una cabecera, lo comprobamos
                    if ((uint8_t) buff_to_process[bufLength - 1] == 0xFA) {
                        buff_to_process.erase(buff_to_process.begin(), buff_to_process.end() - 1);
                    } else {
                        //No hay nada util, borramos todo
                        buff_to_process.clear();
                    }
                    break;
                } else {
                     ROS_DEBUG("DEBUG:Header found HeaderPos: %d", headerPos);
                    //Comprobamos si en el buffer queda suficiente espacio para un mensaje completo
                    bool enoughSize = (headerPos + TOF_REPORT_LEN) <= bufLength;

                    if (enoughSize) {
                         ROS_DEBUG("DEBUG:has enoughSize");
                        //Comprobamos si la cola es correcta
                        bool tailFound = false;
                         ROS_DEBUG("DEBUG:Tail pos byte: %x", (uint8_t) buff_to_process[headerPos + TOF_REPORT_LEN - 2]);
                        if (((uint8_t) buff_to_process[headerPos + TOF_REPORT_LEN - 2] == 0xBB) && ((uint8_t) buff_to_process[headerPos + TOF_REPORT_LEN - 1] == 0xBB)) {
                            tailFound = true;
                            ROS_DEBUG("DEBUG:TAIL FOUND ++++");
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
                        ROS_DEBUG("DEBUG:has NOT enoughSize");
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



void PozyxRangingCIRReader::start(std::string usbPort, ros::Publisher aPub) {

    ROS_INFO("Starting Pozyx USB receiver...");
    uwb_port_name = usbPort;
    ros_pub = aPub;

    ROS_INFO("Opening USB PORT:  %s ...", uwb_port_name.c_str());
    bool uwbPortOpen = (openSerialPort(uwb_port_name, PORT_TYPE_UWB) == 0);

    if (uwbPortOpen) {
        ROS_INFO("Port OPEN.");
        async_read_some_();
        boost::thread t(boost::bind(&boost::asio::io_service::run, &io_service_));

    } else {
        ROS_INFO("Error: Could not open the USB port.");
    }

}


void PozyxRangingCIRReader::newData(const std::vector<char> data) {

    ROS_DEBUG("DEBUG:NEW DATA ****");
    std::vector<int> cirMeasurements;
    int numCirMeasurements = 0;
    std::string nowstr = "T:hhmmsszzz:";
    int originType, destinationType, range, rangetime, seq, prf, channel, datarate, prfValue;
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

    //ROS_DEBUG("DEBUG:seq pos byte: %x",  (uint8_t)data[14]);
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
        numCirMeasurements = 996;
    } else if (prf == DWT_PRF_64M) {
        prfValue = 64;
        numCirMeasurements = 1016;
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


    //CIR
    gtec_msgs::PozyxRangingWithCir ranging_with_cir_msg;

    for (int i = 0; i < 2*numCirMeasurements; ++i)
    {
        int aCirMeasurement = int((uint8_t)(data[22+2*i+1]) << 8 |
                        (uint8_t)(data[22+2*i]));

        ranging_with_cir_msg.cir.push_back(aCirMeasurement);
    }

    ranging_with_cir_msg.originId = originId;
    ranging_with_cir_msg.originType = originType;
    ranging_with_cir_msg.destinationId = destinationId;
    ranging_with_cir_msg.destinationType = destinationType;
    ranging_with_cir_msg.range = range;
    ranging_with_cir_msg.seq = seq;
    ranging_with_cir_msg.ts = rangetime;
    ranging_with_cir_msg.rxPower = rss;
    ranging_with_cir_msg.channel = channelValue;
    ranging_with_cir_msg.prf = prfValue;
    ranging_with_cir_msg.datarate = datarateValue;
    ranging_with_cir_msg.preambleLength = 0;
    ranging_with_cir_msg.txGain = 0;
    ranging_with_cir_msg.angle = angle;

    if (seq <= 255) {
        ros_pub.publish(ranging_with_cir_msg);
    }

}

//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#include "NetworkServerApp.h"
#include "inet/networklayer/ipv4/IPv4Datagram.h"
#include "inet/networklayer/contract/ipv4/IPv4ControlInfo.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/common/ModuleAccess.h"
#include "inet/applications/base/ApplicationPacket_m.h"

namespace inet {

Define_Module(NetworkServerApp);


void NetworkServerApp::initialize(int stage)
{
    if (stage == 0) {
        ASSERT(recvdPackets.size()==0);
        LoRa_ServerPacketReceived = registerSignal("LoRa_ServerPacketReceived");
        localPort = par("localPort");
        destPort = par("destPort");
        adrMethod = par("adrMethod").stdstringValue();
    } else if (stage == INITSTAGE_APPLICATION_LAYER) {
        startUDP();
        getSimulation()->getSystemModule()->subscribe("LoRa_AppPacketSent", this);
        evaluateADRinServer = par("evaluateADRinServer");
        confirmedTrafficMode=par("confirmedTrafficMode");
        adrDeviceMargin = par("adrDeviceMargin");
        receivedRSSI.setName("Received RSSI");
        totalReceivedPackets = 0;
        for(int i=0;i<6;i++)
        {
            counterUniqueReceivedPacketsPerSF[i] = 0;
            counterOfSentPacketsFromNodesPerSF[i] = 0;
        }
    }
}


void NetworkServerApp::startUDP()
{
    socket.setOutputGate(gate("udpOut"));
    const char *localAddress = par("localAddress");
    socket.bind(*localAddress ? L3AddressResolver().resolve(localAddress) : L3Address(), localPort);
}


void NetworkServerApp::handleMessage(cMessage *msg)
{
    if (msg->arrivedOn("udpIn")) {
        LoRaMacFrame *frame = check_and_cast<LoRaMacFrame *>(msg);
      //  EV<<"the value of droppeddc at the NS is "<<frame->getDroppedDC()<<endl;
        if (simTime() >= getSimulation()->getWarmupPeriod())
        {
            totalReceivedPackets++;
        }
        updateKnownNodes(frame);
        updateKnownGateways(frame);
        processLoraMACPacket(PK(msg));
    } else if(msg->isSelfMessage())
    {
        processScheduledPacket(msg);
    }
}

void NetworkServerApp::processLoraMACPacket(cPacket *pk)
{
    LoRaMacFrame *frame = check_and_cast<LoRaMacFrame *>(pk);
    if(isPacketProcessed(frame))
    {EV<<"packet is processed"<<endl;
        delete pk;
        return;
    }
    addPktToProcessingTable(frame);
}

void NetworkServerApp::finish()
{
    recordScalar("LoRa_NS_DER", double(counterUniqueReceivedPackets)/counterOfSentPacketsFromNodes);
    for(uint i=0;i<knownNodes.size();i++)
    {
        delete knownNodes[i].historyAllSNIR;
        delete knownNodes[i].historyAllRSSI;
        delete knownNodes[i].receivedSeqNumber;
        delete knownNodes[i].calculatedSNRmargin;
        recordScalar("Send ADR for node", knownNodes[i].numberOfSentADRPackets);
    }
    for (std::map<int,int>::iterator it=numReceivedPerNode.begin(); it != numReceivedPerNode.end(); ++it)
    {
        const std::string stringScalar = "numReceivedFromNode " + std::to_string(it->first);
        recordScalar(stringScalar.c_str(), it->second);
    }

    receivedRSSI.recordAs("receivedRSSI");
    recordScalar("totalReceivedPackets", totalReceivedPackets);
    for(uint i=0;i<receivedPackets.size();i++)
    {
        delete receivedPackets[i].rcvdPacket;
    }
    recordScalar("counterUniqueReceivedPacketsPerSF SF7", counterUniqueReceivedPacketsPerSF[0]);
    recordScalar("counterUniqueReceivedPacketsPerSF SF8", counterUniqueReceivedPacketsPerSF[1]);
    recordScalar("counterUniqueReceivedPacketsPerSF SF9", counterUniqueReceivedPacketsPerSF[2]);
    recordScalar("counterUniqueReceivedPacketsPerSF SF10", counterUniqueReceivedPacketsPerSF[3]);
    recordScalar("counterUniqueReceivedPacketsPerSF SF11", counterUniqueReceivedPacketsPerSF[4]);
    recordScalar("counterUniqueReceivedPacketsPerSF SF12", counterUniqueReceivedPacketsPerSF[5]);
    if (counterOfSentPacketsFromNodesPerSF[0] > 0)
        recordScalar("DER SF7", double(counterUniqueReceivedPacketsPerSF[0]) / counterOfSentPacketsFromNodesPerSF[0]);
    else
        recordScalar("DER SF7", 0);

    if (counterOfSentPacketsFromNodesPerSF[1] > 0)
        recordScalar("DER SF8", double(counterUniqueReceivedPacketsPerSF[1]) / counterOfSentPacketsFromNodesPerSF[1]);
    else
        recordScalar("DER SF8", 0);

    if (counterOfSentPacketsFromNodesPerSF[2] > 0)
        recordScalar("DER SF9", double(counterUniqueReceivedPacketsPerSF[2]) / counterOfSentPacketsFromNodesPerSF[2]);
    else
        recordScalar("DER SF9", 0);

    if (counterOfSentPacketsFromNodesPerSF[3] > 0)
        recordScalar("DER SF10", double(counterUniqueReceivedPacketsPerSF[3]) / counterOfSentPacketsFromNodesPerSF[3]);
    else
        recordScalar("DER SF10", 0);

    if (counterOfSentPacketsFromNodesPerSF[4] > 0)
        recordScalar("DER SF11", double(counterUniqueReceivedPacketsPerSF[4]) / counterOfSentPacketsFromNodesPerSF[4]);
    else
        recordScalar("DER SF11", 0);

    if (counterOfSentPacketsFromNodesPerSF[5] > 0)
        recordScalar("DER SF12", double(counterUniqueReceivedPacketsPerSF[5]) / counterOfSentPacketsFromNodesPerSF[5]);
    else
        recordScalar("DER SF12", 0);
}

bool NetworkServerApp::isPacketProcessed(LoRaMacFrame* pkt)
{
    for(uint i=0;i<knownNodes.size();i++)
    {
        if(knownNodes[i].srcAddr == pkt->getTransmitterAddress())
        {
            if(knownNodes[i].lastSeqNoProcessed > pkt->getSequenceNumber()) return true;
        }
    }
    return false;
}

void NetworkServerApp::updateKnownNodes(LoRaMacFrame* pkt)
{
    bool nodeExist = false;
    for(uint i=0;i<knownNodes.size();i++)
    {
        if(knownNodes[i].srcAddr == pkt->getTransmitterAddress())
        {
            nodeExist = true;
            if(knownNodes[i].lastSeqNoProcessed < pkt->getSequenceNumber())
            {
                knownNodes[i].lastSeqNoProcessed = pkt->getSequenceNumber();
            }
            break;
        }
    }
    if(nodeExist == false)
    {
        knownNode newNode;
        newNode.srcAddr= pkt->getTransmitterAddress();
        newNode.lastSeqNoProcessed = pkt->getSequenceNumber();
        newNode.framesFromLastADRCommand = 0;
        newNode.numberOfSentADRPackets = 0;
        newNode.historyAllSNIR = new cOutVector;
        newNode.historyAllSNIR->setName("Vector of SNIR per node");
        //newNode.historyAllSNIR->record(pkt->getSNIR());
        newNode.historyAllSNIR->record(math::fraction2dB(pkt->getSNIR()));
        newNode.historyAllRSSI = new cOutVector;
        newNode.historyAllRSSI->setName("Vector of RSSI per node");
        newNode.historyAllRSSI->record(pkt->getRSSI());
        newNode.receivedSeqNumber = new cOutVector;
        newNode.receivedSeqNumber->setName("Received Sequence number");
        newNode.calculatedSNRmargin = new cOutVector;
        newNode.calculatedSNRmargin->setName("Calculated SNRmargin in ADR");
        knownNodes.push_back(newNode);
    }
}

void NetworkServerApp::updateKnownGateways(LoRaMacFrame* pkt)
{    UDPDataIndication *cInfo = check_and_cast<UDPDataIndication*>(pkt->getControlInfo());

    bool nodeExist = false;
    EV<<"the size of the knownGateways is "<<knownGateways.size()<<endl;
    for(uint i=0;i<knownGateways.size();i++)
    {
        if(knownGateways[i].ipAddr == cInfo->getSrcAddr())
        {
            EV<<"IP already in the KnownGateways and the size of the list is  "<<knownGateways[i].sentPcktListTime.size()<<endl;
            nodeExist = true;
            break;
        }
    }
    if(nodeExist == false)
    {
        knownGW newGateway;
        newGateway.ipAddr=cInfo->getSrcAddr();
        EV<<"adding new gateway to the vector of gateways and the ip of this gateway is"<<cInfo->getSrcAddr()<<endl;

        knownGateways.push_back(newGateway);
    }
}
void NetworkServerApp::addPktToProcessingTable(LoRaMacFrame* pkt)
{
    //EV<<"addPktToProcessingTable mmm"<<endl;
   // EV<<"transmtter addr is "<< pkt->getTransmitterAddress().str()<< " and seq is "<<pkt->getSequenceNumber()<<endl;
    bool packetExists = false;
    UDPDataIndication *cInfo = check_and_cast<UDPDataIndication*>(pkt->getControlInfo());
  // EV<<"FINDING DIFFERENCE BETWEEN pkt->getTransmitterAddress and cInfo->getSrcAddr() the first is "<<pkt->getTransmitterAddress()<<" and the second is "<<cInfo->getSrcAddr()<<endl;
    for(uint i=0;i<receivedPackets.size();i++)
    {
        if(receivedPackets[i].rcvdPacket->getTransmitterAddress() == pkt->getTransmitterAddress() && receivedPackets[i].rcvdPacket->getSequenceNumber() == pkt->getSequenceNumber())
        {//EV<<"line 204 packet exist"<<endl;
            packetExists = true;
            receivedPackets[i].possibleGateways.emplace_back(cInfo->getSrcAddr(), math::fraction2dB(pkt->getSNIR()), pkt->getRSSI(), pkt->getEndOfWaitingDC());
            delete pkt;
            break;
        }
    }
    if(packetExists == false)// go throught this code for each unique packet
    {
        receivedPacket rcvPkt;
        rcvPkt.rcvdPacket = pkt;
        rcvPkt.endOfWaiting = new cMessage("endOfWaitingWindow");
        rcvPkt.endOfWaiting->setContextPointer(pkt);
        rcvPkt.possibleGateways.emplace_back(cInfo->getSrcAddr(), math::fraction2dB(pkt->getSNIR()), pkt->getRSSI(), pkt->getEndOfWaitingDC());
        EV<<"Firiing end of waiting msg"<<endl;
        scheduleAt(simTime() + 1.2, rcvPkt.endOfWaiting);
        receivedPackets.push_back(rcvPkt);
    }
}

void NetworkServerApp::processScheduledPacket(cMessage* selfMsg)
{
    LoRaMacFrame *frame = static_cast<LoRaMacFrame *>(selfMsg->getContextPointer());
    if (simTime() >= getSimulation()->getWarmupPeriod())
    {
        counterUniqueReceivedPacketsPerSF[frame->getLoRaSF()-7]++;
    }
   // EV<<"line 232 processScheduledPacket trasmitter addr is "<<frame->getTransmitterAddress().str()<< " and seq is "<<frame->getSequenceNumber()<<endl;
    L3Address pickedGateway;
    double SNIRinGW = -99999999999;
    double RSSIinGW = -99999999999;
    int packetNumber;
    int nodeNumber;
    long receivedDroppedDCCount;
    simtime_t receivedDroppedDCEndTimer=0;
    for(uint i=0;i<receivedPackets.size();i++)
    {
        if(receivedPackets[i].rcvdPacket->getTransmitterAddress() == frame->getTransmitterAddress() && receivedPackets[i].rcvdPacket->getSequenceNumber() == frame->getSequenceNumber())
        {
            packetNumber = i;
            nodeNumber = frame->getTransmitterAddress().getInt();
            if (numReceivedPerNode.count(nodeNumber-1)>0)
            {
                ++numReceivedPerNode[nodeNumber-1];
            } else {
                numReceivedPerNode[nodeNumber-1] = 1;
            }
            std::vector<std::tuple<L3Address, double>> gwEfecieciencies;
            for(uint j=0;j<receivedPackets[i].possibleGateways.size();j++)
            {   // DroppedDCCount by gateway
             //  receivedDroppedDCCount=std::get<3>(receivedPackets[i].possibleGateways[j]);
              // receivedDroppedDCEndTimer=std::get<3>(receivedPackets[i].possibleGateways[j]);

              // EV<<"receivedDroppedDCEndTimer from gateway "<<std::get<0>(receivedPackets[i].possibleGateways[j])<<" is "<<receivedDroppedDCEndTimer<<endl;
              // EV<<"receivedDroppedDCEndTimer from gateway "<<std::get<0>(receivedPackets[i].possibleGateways[j])<<" is "<<std::get<3>(receivedPackets[i].possibleGateways[j])<<endl;
              // L3Address findingpossibleGateway=std::get<0>(receivedPackets[i].possibleGateways[j]);
              /*  for(uint k=0;k<knownGateways.size();k++)
                    {
                        if(knownGateways[k].ipAddr == findingpossibleGateway)
                        {
                            EV<<" Number of sent packets to this gateway is "<<knownGateways[k].sentPcktListTime.size()<<endl;
                            // loop inside sentPcktListTime to find how many items there that their values is equal
                            // to current time or their values  grater than current time -600 s for example.
                            double  counterfornumberOfUsingThisGatewayForLastInterval=0;
                            for (std::list<simtime_t>::iterator it=knownGateways[k].sentPcktListTime.begin(); it != knownGateways[k].sentPcktListTime.end(); ++it)
                                                {
                                                   if (*it > (simTime()-10)){
                                                       counterfornumberOfUsingThisGatewayForLastInterval++;
                                                   }
                                                }
                            EV<<"Number of using this gateway in the last 600 second is "<<counterfornumberOfUsingThisGatewayForLastInterval<<endl;
                            double resultSub=counterfornumberOfUsingThisGatewayForLastInterval- (double)receivedDroppedDCCount;
                            double gwEffeciency=resultSub/counterfornumberOfUsingThisGatewayForLastInterval;
                           EV<<"The result of subtraction is "<<resultSub<<" and the result of the division is "<< resultSub/counterfornumberOfUsingThisGatewayForLastInterval<<endl;
                           //gwEfecieciencies.emplace_back(findingpossibleGateway,gwEffeciency);
                           gwEfecieciencies.emplace_back(findingpossibleGateway, (double)receivedDroppedDCCount);

                        }
                    }*/
                //this part is used to select  gateway based on the SNR
             /*  if(SNIRinGW < std::get<1>(receivedPackets[i].possibleGateways[j]))
                {

                    RSSIinGW = std::get<2>(receivedPackets[i].possibleGateways[j]);
                    SNIRinGW = std::get<1>(receivedPackets[i].possibleGateways[j]);
                    pickedGateway = std::get<0>(receivedPackets[i].possibleGateways[j]);
                }*/
                // this part is used to select  gateway based on the end of DC timer

              if(receivedDroppedDCEndTimer > std::get<3>(receivedPackets[i].possibleGateways[j])||receivedDroppedDCEndTimer==0)
                                {
                                    receivedDroppedDCEndTimer=std::get<3>(receivedPackets[i].possibleGateways[j]);
                                    RSSIinGW = std::get<2>(receivedPackets[i].possibleGateways[j]);
                                    SNIRinGW = std::get<1>(receivedPackets[i].possibleGateways[j]);
                                    pickedGateway = std::get<0>(receivedPackets[i].possibleGateways[j]);
                                }
            }
            //EV<<"the SNIRinGW based on SNR mechansism is  "<<RSSIinGW<<endl;
            //EV<<"the RSSIinGW  based on SNR mechansism is "<<SNIRinGW<<endl;
           // std::list<double> effeciencies;
           // for (uint n=0;n<gwEfecieciencies.size();n++){
              //  EV<<"GW IP: "<<std::get<0>(gwEfecieciencies[n])<<" Effeciency: "<<std::get<1>(gwEfecieciencies[n])<<endl;
              //  effeciencies.push_back(std::get<1>(gwEfecieciencies[n]));
           // }
           // EV<<"The highest effeciency is "<<*max_element(effeciencies.begin(), effeciencies.end())<<endl;;
            //EV<<"The highest effeciency is "<<*min_element(effeciencies.begin(), effeciencies.end())<<endl;;

            //double maxEffeciency=*max_element(effeciencies.begin(), effeciencies.end());
           // double minEffeciency=*min_element(effeciencies.begin(), effeciencies.end());

            //L3Address chosenGWBeasedOnEffeciency;
          //  EV<<"The chosen gateway based on SNR is "<<pickedGateway<<endl;
          /*  if (gwEfecieciencies.size()>1){
                for (uint m=0;m<gwEfecieciencies.size();m++){
                   // if (std::get<1>(gwEfecieciencies[m])==maxEffeciency){
                        if (std::get<1>(gwEfecieciencies[m])==minEffeciency){
                        chosenGWBeasedOnEffeciency=std::get<0>(gwEfecieciencies[m]);
                    }
                }
            // loop inside possibleGateways
               for (uint j=0;j<receivedPackets[i].possibleGateways.size();j++){
                    if (std::get<0>(receivedPackets[i].possibleGateways[j])==chosenGWBeasedOnEffeciency){
                        RSSIinGW = std::get<2>(receivedPackets[i].possibleGateways[j]);
                        SNIRinGW = std::get<1>(receivedPackets[i].possibleGateways[j]);
                        pickedGateway = std::get<0>(receivedPackets[i].possibleGateways[j]);
                    }
                }
            }*/
            //EV<<"The chosen gateway based on effeciency is "<<pickedGateway<<endl;
           // EV<<"the SNIRinGW is 340 "<<RSSIinGW<<endl;
          //  EV<<"the RSSIinGW is 340 "<<SNIRinGW<<endl;
        }
    }
    emit(LoRa_ServerPacketReceived, true);
    if (simTime() >= getSimulation()->getWarmupPeriod())
    {
        counterUniqueReceivedPackets++;
    }
    receivedRSSI.collect(frame->getRSSI());
    if(evaluateADRinServer)
    {
        EV << "calling evaluate ADR method" << endl;
        evaluateADR(frame, pickedGateway, SNIRinGW, RSSIinGW);
    }
    if (confirmedTrafficMode){
        // call a method to instantiaate the ack reply
        sendAck(frame, pickedGateway, SNIRinGW, RSSIinGW);
    }
    delete receivedPackets[packetNumber].rcvdPacket;
    delete selfMsg;
    receivedPackets.erase(receivedPackets.begin()+packetNumber);
}
void NetworkServerApp::sendAck(LoRaMacFrame* pkt, L3Address pickedGateway, double SNIRinGW, double RSSIinGW)
{
    //EV<<"inside sendAck "<<endl;
   // EV<<"ack for device address"<< pkt->getTransmitterAddress().str()<< " and for packet with seq no. "<<pkt->getSequenceNumber()<<endl;
    LoRaAppPacket *mgmtPacketAck = new LoRaAppPacket("ACKcommand");
    mgmtPacketAck->setMsgType(TXACK);
    mgmtPacketAck->setSequenceNumberAppLayer(pkt->getSequenceNumber());
    LoRaMacFrame *frameToSendAck = new LoRaMacFrame("ACKPacket");
    frameToSendAck->encapsulate(mgmtPacketAck);
    frameToSendAck->setReceiverAddress(pkt->getTransmitterAddress());
            //FIXME: What value to set for LoRa TP
            //frameToSend->setLoRaTP(pkt->getLoRaTP());
    frameToSendAck->setLoRaTP(14);
    frameToSendAck->setLoRaCF(pkt->getLoRaCF());
    frameToSendAck->setLoRaSF(pkt->getLoRaSF());
    frameToSendAck->setLoRaBW(pkt->getLoRaBW());
   // loop to find the chosen knowngw, then insert current simulation time into its sentPcktListTime list
    /*for(uint i=0;i<knownGateways.size();i++)
       {
           if(knownGateways[i].ipAddr == pickedGateway)
           {
               EV<<"found the pickedGateway 332 "<<endl;
               knownGateways[i].sentPcktListTime.push_back(simTime());
           }
       }*/
   // EV<<" 395 Byte length is "<<frameToSendAck->getByteLength()<<endl;

    socket.sendTo(frameToSendAck, pickedGateway, destPort);
}
void NetworkServerApp::evaluateADR(LoRaMacFrame* pkt, L3Address pickedGateway, double SNIRinGW, double RSSIinGW)
{
    bool sendADR = false;
    bool sendADRAckRep = false;
    double SNRm; //needed for ADR
    int nodeIndex;

    LoRaAppPacket *rcvAppPacket = check_and_cast<LoRaAppPacket*>(pkt->decapsulate());
    if(rcvAppPacket->getOptions().getADRACKReq())
    {
        sendADRAckRep = true;
    }

    for(uint i=0;i<knownNodes.size();i++)
    {
        if(knownNodes[i].srcAddr == pkt->getTransmitterAddress())
        {
            knownNodes[i].adrListSNIR.push_back(SNIRinGW);
            knownNodes[i].historyAllSNIR->record(SNIRinGW);
            knownNodes[i].historyAllRSSI->record(RSSIinGW);
            knownNodes[i].receivedSeqNumber->record(pkt->getSequenceNumber());
            if(knownNodes[i].adrListSNIR.size() == 20) knownNodes[i].adrListSNIR.pop_front();
            knownNodes[i].framesFromLastADRCommand++;

            if(knownNodes[i].framesFromLastADRCommand == 20)
            {
                nodeIndex = i;
                knownNodes[i].framesFromLastADRCommand = 0;
                sendADR = true;
                if(adrMethod == "max")
                {
                    SNRm = *max_element(knownNodes[i].adrListSNIR.begin(), knownNodes[i].adrListSNIR.end());
                }
                if(adrMethod == "avg")
                {
                    double totalSNR = 0;
                    int numberOfFields = 0;
                    for (std::list<double>::iterator it=knownNodes[i].adrListSNIR.begin(); it != knownNodes[i].adrListSNIR.end(); ++it)
                    {
                        totalSNR+=*it;
                        numberOfFields++;
                    }
                    SNRm = totalSNR/numberOfFields;
                }

            }

        }
    }

    if(sendADR || sendADRAckRep)
    {
        LoRaAppPacket *mgmtPacket = new LoRaAppPacket("ADRcommand");
        mgmtPacket->setMsgType(TXCONFIG);

        if(sendADR)
        {
            double SNRmargin;
            double requiredSNR;
            if(pkt->getLoRaSF() == 7) requiredSNR = -7.5;
            if(pkt->getLoRaSF() == 8) requiredSNR = -10;
            if(pkt->getLoRaSF() == 9) requiredSNR = -12.5;
            if(pkt->getLoRaSF() == 10) requiredSNR = -15;
            if(pkt->getLoRaSF() == 11) requiredSNR = -17.5;
            if(pkt->getLoRaSF() == 12) requiredSNR = -20;

            SNRmargin = SNRm - requiredSNR - adrDeviceMargin;
            knownNodes[nodeIndex].calculatedSNRmargin->record(SNRmargin);
            int Nstep = round(SNRmargin/3);
            LoRaOptions newOptions;

            // Increase the data rate with each step
            int calculatedSF = pkt->getLoRaSF();
            while(Nstep > 0 && calculatedSF > 7)
            {
                calculatedSF--;
                Nstep--;
            }

            // Decrease the Tx power by 3 for each step, until min reached
            double calculatedPowerdBm = pkt->getLoRaTP();
            while(Nstep > 0 && calculatedPowerdBm > 2)
            {
                calculatedPowerdBm-=3;
                Nstep--;
            }
            if(calculatedPowerdBm < 2) calculatedPowerdBm = 2;

            // Increase the Tx power by 3 for each step, until max reached
            while(Nstep < 0 && calculatedPowerdBm < 14)
            {
                calculatedPowerdBm+=3;
                Nstep++;
            }
            if(calculatedPowerdBm > 14) calculatedPowerdBm = 14;

            newOptions.setLoRaSF(calculatedSF);
            newOptions.setLoRaTP(calculatedPowerdBm);
            mgmtPacket->setOptions(newOptions);
        }

        if(simTime() >= getSimulation()->getWarmupPeriod())
        {
            knownNodes[nodeIndex].numberOfSentADRPackets++;
        }

        LoRaMacFrame *frameToSend = new LoRaMacFrame("ADRPacket");
        frameToSend->encapsulate(mgmtPacket);
        frameToSend->setReceiverAddress(pkt->getTransmitterAddress());
        //FIXME: What value to set for LoRa TP
        //frameToSend->setLoRaTP(pkt->getLoRaTP());
        frameToSend->setLoRaTP(14);
        frameToSend->setLoRaCF(pkt->getLoRaCF());
        frameToSend->setLoRaSF(pkt->getLoRaSF());
        frameToSend->setLoRaBW(pkt->getLoRaBW());
        // loop to find the chosen knowngw, then insert current simulation time into its sentPcktListTime list
            /*for(uint i=0;i<knownGateways.size();i++)
               {
                   if(knownGateways[i].ipAddr == pickedGateway)
                   {
                       EV<<"found the pickedGateway 332 "<<endl;
                       knownGateways[i].sentPcktListTime.push_back(simTime());
                   }
               }*/
            //EV<<"Byte length is "<<frameToSend->getByteLength()<<endl;
        socket.sendTo(frameToSend, pickedGateway, destPort);
    }

    delete rcvAppPacket;
}

void NetworkServerApp::receiveSignal(cComponent *source, simsignal_t signalID, long value, cObject *details)
{
EV<<"NetworkServerApp::receiveSignal and signal is "<<signalID<< " and signal name is "<<getSignalName(signalID)<<endl;
    if (simTime() >= getSimulation()->getWarmupPeriod())
    {
        counterOfSentPacketsFromNodes++;
        counterOfSentPacketsFromNodesPerSF[value-7]++;
    }
}

} //namespace inet

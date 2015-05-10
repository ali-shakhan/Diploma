// PN532 library by adafruit/ladyada
// MIT license

// authenticateBlock, readMemoryBlock, writeMemoryBlock contributed
// by Seeed Technology Inc (www.seeedstudio.com)

#include "PN532.h"

#define PN532DEBUG 1

uint8_t pn532ack[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
uint8_t pn532response_firmwarevers[] = {0x00, 0xFF, 0x06, 0xFA, 0xD5, 0x03};

#define COMMAND_RESPONSE_SIZE 3
#define TS_GET_DATA_IN_MAX_SIZE  262 + 3

uint8_t pn532_packetbuffer[TS_GET_DATA_IN_MAX_SIZE];

PN532::PN532(uint8_t clk, uint8_t miso, uint8_t mosi, uint8_t ss)
{
    _clk = clk;
    _miso = miso;
    _mosi = mosi;
    _ss = ss;

    pinMode(_ss, OUTPUT);
    pinMode(_clk, OUTPUT);
    pinMode(_mosi, OUTPUT);
    pinMode(_miso, INPUT);
}

void PN532::initializeReader()
{
    digitalWrite(_ss, LOW);

    delay(1000);

    // not exactly sure why but we have to send a dummy command to get synced up
    pn532_packetbuffer[0] = PN532_FIRMWAREVERSION;
    sendCommandCheckAck(pn532_packetbuffer, 1);

    // ignore response!
}


uint32_t PN532::getFirmwareVersion(void)
{
    uint32_t version;

    pn532_packetbuffer[0] = PN532_FIRMWAREVERSION;

    if (IS_ERROR(sendCommandCheckAck(pn532_packetbuffer, 1)))
    {
        return 0;
    }

    // read response Packet
    PN532_CMD_RESPONSE *response = (PN532_CMD_RESPONSE *) pn532_packetbuffer;
    if (IS_ERROR(readspicommand(PN532_FIRMWAREVERSION, response)))
    {
       return 0;
    }

    //response->printResponse();

    version = response->data[0];
    version <<= 8;
    version |= response->data[1];
    version <<= 8;
    version |= response->data[2];
    version <<= 8;
    version |= response->data[3];

    return version;
}


// default timeout of one second
uint32_t PN532::sendCommandCheckAck(uint8_t *cmd,
                                    uint8_t cmdlen,
                                    uint16_t timeout,
                                    boolean debug)
{
    uint16_t timer = 0;

    // write the command
    spiwritecommand(cmd, cmdlen, debug);

    // Wait for chip to say its ready!
    while (readspistatus() != PN532_SPI_READY)
    {
        if (timeout != 0)
        {
            timer+=10;
            if (timer > timeout)
            {
                return SEND_COMMAND_TX_TIMEOUT_ERROR;
            }
        }
        delay(10);
    }

    // read acknowledgement
    if (!spi_readack(debug)) {
        return SEND_COMMAND_RX_ACK_ERROR;
    }

    timer = 0;

    timeout = 3000;
    // Wait for chip to say its ready!
    while (readspistatus() != PN532_SPI_READY)
    {
        if (timeout != 0)
        {
            timer+=10;
            if (timer > timeout)
            {
#ifdef NDEF_DEBUG
                if (debug)
                {
                    Serial.println(F("sendCommandCheckAck(): time out when waiting for chip ready"));
                }
#endif
				return SEND_COMMAND_RX_TIMEOUT_ERROR;
            }
        }
        delay(10);
    }

    return RESULT_SUCCESS; // ack'd command
}

uint32_t PN532::SAMConfig(void)
{
    pn532_packetbuffer[0] = PN532_SAMCONFIGURATION;
    pn532_packetbuffer[1] = 0x01; // normal mode;
    pn532_packetbuffer[2] = 0x14; // timeout 50ms * 20 = 1 second
    pn532_packetbuffer[3] = 0x01; // use IRQ pin!

    uint32_t result = sendCommandCheckAck(pn532_packetbuffer, 4);

    if (IS_ERROR(result))
    {
        return result;
    }

    // read data packet
    PN532_CMD_RESPONSE *response = (PN532_CMD_RESPONSE *) pn532_packetbuffer;
    return readspicommand(PN532_SAMCONFIGURATION, response);
}

uint32_t PN532::configurePeerAsInitiator(uint8_t baudrate)
{
    // Does not support 106
    if (baudrate != NFC_READER_CFG_BAUDRATE_201_KPS && baudrate != NFC_READER_CFG_BAUDRATE_424_KPS)
    {
       return CONFIGURE_HARDWARE_ERROR;
    }

    pn532_packetbuffer[0] = PN532_INJUMPFORDEP;
    pn532_packetbuffer[1] = 0x01; //Active Mode
    pn532_packetbuffer[2] = baudrate; // Use 1 or 2. //0 i.e 106kps is not supported yet
    pn532_packetbuffer[3] = 0x01; //Indicates Optional Payload is present

    //Polling request payload
    pn532_packetbuffer[4] = 0x00;
    pn532_packetbuffer[5] = 0xFF;
    pn532_packetbuffer[6] = 0xFF;
    pn532_packetbuffer[7] = 0x00;
    pn532_packetbuffer[8] = 0x00;

    uint32_t result = sendCommandCheckAck(pn532_packetbuffer, 9);

    if (IS_ERROR(result))
    {
        return result;
    }

    // read data packet
    PN532_CMD_RESPONSE *response = (PN532_CMD_RESPONSE *) pn532_packetbuffer;
    result = readspicommand(PN532_INJUMPFORDEP,  response);
    if (IS_ERROR(result))
    {
       return result;
    }

    if (response->data[0] != 0x00)
    {
       return (GEN_ERROR | response->data[0]);
    }

    return RESULT_SUCCESS; //No error
}


uint32_t PN532::initiatorTxRxData(uint8_t *DataOut,
                           uint32_t dataSize,
                           uint8_t *DataIn,
                           boolean debug)
{
    pn532_packetbuffer[0] = PN532_INDATAEXCHANGE;
    pn532_packetbuffer[1] = 0x01; //Target 01

    for(uint8_t iter=(2+0);iter<(2+dataSize);iter++)
    {
        pn532_packetbuffer[iter] = DataOut[iter-2]; //pack the data to send to target
    }

    uint32_t result = sendCommandCheckAck(pn532_packetbuffer, dataSize+2);

    if (IS_ERROR(result))
    {
        return result;
    }

    PN532_CMD_RESPONSE *response = (PN532_CMD_RESPONSE *) pn532_packetbuffer;
    if (!readspicommand(PN532_INDATAEXCHANGE, response))
    {
       return false;
    }

#ifdef NDEF_DEBUG
    if (debug)
    {
       response->printResponse();
    }
#endif

    if (response->data[0] != 0x00)
    {
       return (GEN_ERROR | response->data[0]);
    }
    return RESULT_SUCCESS; //No error
}

uint32_t PN532::configurePeerAsTarget(uint8_t type)
{
    static const uint8_t snep_client[44] =      { PN532_TGINITASTARGET,
                             0x00,
                             0x00, 0x00, //SENS_RES
                             0x00, 0x00, 0x00, //NFCID1
                             0x00, //SEL_RES

                             0x01, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // POL_RES
                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

                             0x00, 0x00,

                             0x01, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //NFCID3t: Change this to desired value

                             0x06, 0x46,  0x66, 0x6D, 0x01, 0x01, 0x10, 0x00
                             };

    static const uint8_t snep_server[44] =      { PN532_TGINITASTARGET,
                             0x01,
                             0x00, 0x00, //SENS_RES
                             0x00, 0x00, 0x00, //NFCID1
                             0x40, //SEL_RES

                             0x01, 0xFE, 0x0F, 0xBB, 0xBA, 0xA6, 0xC9, 0x89, // POL_RES
                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

                             0xFF, 0xFF,

                             0x01, 0xFE, 0x0F, 0xBB, 0xBA, 0xA6, 0xC9, 0x89, 0x00, 0x00, //NFCID3t: Change this to desired value

                             0x06, 0x46,  0x66, 0x6D, 0x01, 0x01, 0x10, 0x00
                             };

    if (type == SNEP_CLIENT)
    {
       for(uint8_t iter = 0;iter < 44;iter++)
       {
          pn532_packetbuffer[iter] = snep_client[iter];
       }
    }
    else if (type == SNEP_SERVER)
    {
       for(uint8_t iter = 0;iter < 44;iter++)
       {
          pn532_packetbuffer[iter] = snep_server[iter];
       }
    }

    uint32_t result;
    result = sendCommandCheckAck(pn532_packetbuffer, 44);

    if (IS_ERROR(result))
    {
        return result;
    }

    PN532_CMD_RESPONSE *response = (PN532_CMD_RESPONSE *) pn532_packetbuffer;
    return readspicommand(PN532_TGINITASTARGET, response);
}

uint32_t PN532::getTargetStatus(uint8_t *DataIn)
{
    pn532_packetbuffer[0] = PN532_TGTARGETSTATUS;

    if (IS_ERROR(sendCommandCheckAck(pn532_packetbuffer, 1))) {
        return 0;
    }

    // read data packet
    PN532_CMD_RESPONSE *response = (PN532_CMD_RESPONSE *) pn532_packetbuffer;
    if (RESULT_OK(readspicommand(PN532_TGTARGETSTATUS, response)))
    {
       memcpy(DataIn, response->data, response->data_len);
       return response->data_len;
    }

    return 0;
}

uint32_t PN532::targetRxData(uint8_t *DataIn, boolean debug)
{
    ///////////////////////////////////// Receiving from Initiator ///////////////////////////
    pn532_packetbuffer[0] = PN532_TGGETDATA;
    uint32_t result = sendCommandCheckAck(pn532_packetbuffer, 1, 1000, debug);
    if (IS_ERROR(result)) {
        //Serial.println(F("SendCommandCheck Ack Failed"));
        return NFC_READER_COMMAND_FAILURE;
    }

    // read data packet
    PN532_CMD_RESPONSE *response = (PN532_CMD_RESPONSE *) pn532_packetbuffer;

    result = readspicommand(PN532_TGGETDATA, response, debug);

    if (IS_ERROR(result))
    {
       return NFC_READER_RESPONSE_FAILURE;
    }

    if (response->data[0] == 0x00)
    {
       uint32_t ret_len = response->data_len - 1;
       memcpy(DataIn, &(response->data[1]), ret_len);
       return ret_len;
    }

    return (GEN_ERROR | response->data[0]);
}



uint32_t PN532::targetTxData(uint8_t *DataOut, uint32_t dataSize, boolean debug)
{
    ///////////////////////////////////// Sending to Initiator ///////////////////////////
    pn532_packetbuffer[0] = PN532_TGSETDATA;
    uint8_t commandBufferSize = (1 + dataSize);
    for(uint8_t iter=(1+0);iter < commandBufferSize; ++iter)
    {
        pn532_packetbuffer[iter] = DataOut[iter-1]; //pack the data to send to target
    }

    uint32_t result = sendCommandCheckAck(pn532_packetbuffer, commandBufferSize, 1000, debug);
    if (IS_ERROR(result)) {
#ifdef NDEF_DEBUG
        if (debug) {
		  Serial.println(F("TX_Target Command Failed."));
		}
#endif
        return result;
    }


    // read data packet
    PN532_CMD_RESPONSE *response = (PN532_CMD_RESPONSE *) pn532_packetbuffer;

    result = readspicommand(PN532_TGSETDATA, response);
    if (IS_ERROR(result))
    {
       return result;
    }

    if (response->data[0] != 0x00)
    {
       return (GEN_ERROR | response->data[0]);
    }
    return RESULT_SUCCESS; //No error
}


uint32_t PN532::authenticateBlock(uint8_t cardnumber /*1 or 2*/,
                                  uint32_t cid /*Card NUID*/,
                                  uint8_t blockaddress /*0 to 63*/,
                                  uint8_t authtype/*Either KEY_A or KEY_B */,
                                  uint8_t * keys)
{
    pn532_packetbuffer[0] = PN532_INDATAEXCHANGE;
    pn532_packetbuffer[1] = cardnumber;  // either card 1 or 2 (tested for card 1)
    if(authtype == KEY_A)
    {
        pn532_packetbuffer[2] = PN532_AUTH_WITH_KEYA;
    }
    else
    {
        pn532_packetbuffer[2] = PN532_AUTH_WITH_KEYB;
    }
    pn532_packetbuffer[3] = blockaddress; //This address can be 0-63 for MIFARE 1K card

    pn532_packetbuffer[4] = keys[0];
    pn532_packetbuffer[5] = keys[1];
    pn532_packetbuffer[6] = keys[2];
    pn532_packetbuffer[7] = keys[3];
    pn532_packetbuffer[8] = keys[4];
    pn532_packetbuffer[9] = keys[5];

    pn532_packetbuffer[10] = ((cid >> 24) & 0xFF);
    pn532_packetbuffer[11] = ((cid >> 16) & 0xFF);
    pn532_packetbuffer[12] = ((cid >> 8) & 0xFF);
    pn532_packetbuffer[13] = ((cid >> 0) & 0xFF);

    uint32_t result = sendCommandCheckAck(pn532_packetbuffer, 14);

    if (IS_ERROR(result))
    {
        return result;
    }

    PN532_CMD_RESPONSE *response = (PN532_CMD_RESPONSE *) pn532_packetbuffer;
    result = readspicommand(PN532_INDATAEXCHANGE, response);
    if (IS_ERROR(result))
    {
       return result;
    }


    if((response->data[0] == 0x41) && (response->data[1] == 0x00))
    {
  	    return RESULT_SUCCESS;
    }

    return (GEN_ERROR | response->data[1]);
}

uint32_t PN532::readMemoryBlock(uint8_t cardnumber /*1 or 2*/,
                                uint8_t blockaddress /*0 to 63*/,
                                uint8_t * block)
{
    pn532_packetbuffer[0] = PN532_INDATAEXCHANGE;
    pn532_packetbuffer[1] = cardnumber;  // either card 1 or 2 (tested for card 1)
    pn532_packetbuffer[2] = PN532_MIFARE_READ;
    pn532_packetbuffer[3] = blockaddress; //This address can be 0-63 for MIFARE 1K card

    uint32_t result = sendCommandCheckAck(pn532_packetbuffer, 4);

    if (IS_ERROR(result)) {
        return result;
    }

    // read data packet
    PN532_CMD_RESPONSE *response = (PN532_CMD_RESPONSE *) pn532_packetbuffer;
    result = readspicommand(PN532_INDATAEXCHANGE, response);
    if (IS_ERROR(result))
    {
       return result;
    }

    if((response->data[0] == 0x41) && (response->data[1] == 0x00))
    {
  	    return RESULT_SUCCESS; //read successful
    }

    return (GEN_ERROR | response->data[1]);
}

//Do not write to Sector Trailer Block unless you know what you are doing.
uint32_t PN532::writeMemoryBlock(uint8_t cardnumber /*1 or 2*/,
                                 uint8_t blockaddress /*0 to 63*/,
                                 uint8_t * block)
{
    pn532_packetbuffer[0] = PN532_INDATAEXCHANGE;
    pn532_packetbuffer[1] = cardnumber;  // either card 1 or 2 (tested for card 1)
    pn532_packetbuffer[2] = PN532_MIFARE_WRITE;
    pn532_packetbuffer[3] = blockaddress;

    for(uint8_t i =0; i <16; i++)
    {
        pn532_packetbuffer[4+i] = block[i];
    }

    uint32_t result = sendCommandCheckAck(pn532_packetbuffer, 20);

    if (IS_ERROR(result))
    {
        return result;
    }

    // read data packet
    PN532_CMD_RESPONSE *response = (PN532_CMD_RESPONSE *) pn532_packetbuffer;
    result = readspicommand(PN532_INDATAEXCHANGE, response);
    if (IS_ERROR(result))
    {
       return result;
    }

    if((response->data[0] == 0x41) && (response->data[1] == 0x00))
    {
  	    return RESULT_SUCCESS; //read successful
    }

    return (GEN_ERROR | response->data[1]);
}
uint32_t PN532::readPassiveTargetID(uint8_t cardbaudrate)
{
    uint32_t cid;

    pn532_packetbuffer[0] = PN532_INLISTPASSIVETARGET;
    pn532_packetbuffer[1] = 1;  // max 1 cards at once (we can set this to 2 later)
    pn532_packetbuffer[2] = cardbaudrate;

    if (IS_ERROR(sendCommandCheckAck(pn532_packetbuffer, 3)))
    {
        return 0;  // no cards read
    }

    // read data packet
    PN532_CMD_RESPONSE *response = (PN532_CMD_RESPONSE *) pn532_packetbuffer;
    if (IS_ERROR(readspicommand(PN532_INDATAEXCHANGE, response)))
    {
       return 0;
    }

#ifdef NDEF_DEBUG
    // check some basic stuff
    Serial.print(F("Found "));
    Serial.print(response->data[2], DEC);
    Serial.println(F(" tags"));
#endif

    if (response->data[2] != 1)
    {
        return 0;
    }

    uint16_t sens_res = response->data[4];
    sens_res <<= 8;
    sens_res |= response->data[5];

#ifdef NDEF_DEBUG
    Serial.print(F("Sens Response: 0x"));
    Serial.println(sens_res, HEX);
    Serial.print(F("Sel Response: 0x"));
    Serial.println(response->data[6], HEX);
#endif

    cid = 0;
    for (uint8_t i = 0; i < response->data[7]; i++)
    {
        cid <<= 8;
        cid |= response->data[8 + i];
#ifdef NDEF_DEBUG
        Serial.print(F(" 0x"));
        Serial.print(response->data[8 + i], HEX);
#endif
	}

    return cid;
}

uint32_t PN532::readPassiveTargetID(uint8_t cardbaudrate, uint8_t * uid, uint8_t * uidLength) {
  pn532_packetbuffer[0] = PN532_INLISTPASSIVETARGET;
  pn532_packetbuffer[1] = 1;  // max 1 cards at once (we can set this to 2 later)
  pn532_packetbuffer[2] = cardbaudrate;
  
  if (! sendCommandCheckAck(pn532_packetbuffer, 3))
    return 0x0;  // no cards read
  
  // read data packet
  readspidata(pn532_packetbuffer, 20);
  // check some basic stuff

  /* ISO14443A card response should be in the following format:
  
    byte            Description
    -------------   ------------------------------------------
    b0..6           Frame header and preamble
    b7              Tags Found
    b8              Tag Number (only one used in this example)
    b9..10          SENS_RES
    b11             SEL_RES
    b12             NFCID Length
    b13..NFCIDLen   NFCID                                      */
  
#ifdef MIFAREDEBUG
    Serial.print("Found "); Serial.print(pn532_packetbuffer[7], DEC); Serial.println(" tags");
#endif
  if (pn532_packetbuffer[7] != 1) 
    return 0;
    
  uint16_t sens_res = pn532_packetbuffer[9];
  sens_res <<= 8;
  sens_res |= pn532_packetbuffer[10];
#ifdef MIFAREDEBUG
    Serial.print("ATQA: 0x");  Serial.println(sens_res, HEX); 
    Serial.print("SAK: 0x");  Serial.println(pn532_packetbuffer[11], HEX); 
#endif
  
  /* Card appears to be Mifare Classic */
  *uidLength = pn532_packetbuffer[12];
#ifdef MIFAREDEBUG
    Serial.print("UID:"); 
#endif
  for (uint8_t i=0; i < pn532_packetbuffer[12]; i++) 
  {
    uid[i] = pn532_packetbuffer[13+i];
#ifdef MIFAREDEBUG
      Serial.print(" 0x");Serial.print(uid[i], HEX); 
#endif
  }
#ifdef MIFAREDEBUG
    Serial.println();
#endif

  return 1;
}


inline boolean PN532::isTargetReleasedError(uint32_t result)
{
   return result == (GEN_ERROR | TARGET_RELEASED_ERROR);
}

/************** high level SPI */


boolean PN532::spi_readack(boolean debug)
{
    uint8_t ackbuff[6];

    readspidata(ackbuff, 6, debug);

    return (0 == strncmp((char *)ackbuff, (char *)pn532ack, 6));
}

/************** mid level SPI */

uint8_t PN532::readspistatus(void)
{
    digitalWrite(_ss, LOW);
    delay(2);
    spiwrite(PN532_SPI_STATREAD);
    // read uint8_t
    uint8_t x = spiread();

    digitalWrite(_ss, HIGH);
    return x;
}

uint32_t PN532::readspicommand(uint8_t cmdCode, PN532_CMD_RESPONSE *response, boolean debug)
{

    uint8_t calc_checksum = 0;
    uint8_t ret_checksum;

    digitalWrite(_ss, LOW);
    delay(2);
    spiwrite(PN532_SPI_DATAREAD);

    response->header[0] = response->header[1] = 0xAA;

    uint32_t retVal = RESULT_SUCCESS;

    do
    {
       response->header[0] = response->header[1];
       delay(1);
       response->header[1] = spiread();
    } while (response->header[0] != 0x00 || response->header[1] != 0xFF);

    delay(1);
    response->len = spiread();

    delay(1);
    response->len_chksum = spiread();

    delay(1);
    response->direction = spiread();
    calc_checksum += response->direction;

    delay(1);
    response->responseCode = spiread();

    calc_checksum += response->responseCode;

    retVal = response->verifyResponse(cmdCode) ? RESULT_SUCCESS : INVALID_RESPONSE;

    if (RESULT_OK(retVal))
    {
        // Readjust the len to account only for the data
        // Removing the Direction and response byte from the data length parameter
        response->data_len = response->len - 2;

        for (uint8_t i = 0; i < response->data_len; ++i)
        {
            delay(1);
            response->data[i] = spiread();
            calc_checksum +=  response->data[i];
         }

         delay(1);
         ret_checksum = spiread();

         if (((uint8_t)(calc_checksum + ret_checksum)) != 0x00)
         {
#ifdef NDEF_DEBUG
            Serial.println(F("Invalid Checksum recievied."));
#endif
			retVal = INVALID_CHECKSUM_RX;
         }

         delay(1);
         uint8_t postamble = spiread();


         if (RESULT_OK(retVal) && postamble != 0x00)
         {
             retVal = INVALID_POSTAMBLE;
#ifdef NDEF_DEBUG
             Serial.println(F("Invalid Postamble."));
#endif
		 }

    }

    digitalWrite(_ss, HIGH);
    if (debug)
    {
        response->printResponse();
    }

    return retVal;
}

void PN532::readspidata(uint8_t* buff, uint32_t n, boolean debug)
{
    digitalWrite(_ss, LOW);
    delay(2);
    spiwrite(PN532_SPI_DATAREAD);

#ifdef NDEF_DEBUG
    if (debug)
    {
        Serial.print(F("read:  "));
    }
#endif

    for (uint8_t i=0; i<n; i++)
    {
        delay(1);
        buff[i] = spiread();

        if (debug)
        {
            Serial.print(F(" "));
            Serial.print(buff[i], HEX);
        }
    }

    if (debug)
    {
        Serial.println();
    }

    digitalWrite(_ss, HIGH);
}

void PN532::spiwritecommand(uint8_t* cmd, uint8_t cmdlen, boolean debug)
{
    uint8_t checksum;
    cmdlen++;

#ifdef NDEF_DEBUG
    if (debug)
    {
      Serial.print(F("\nwrite: "));
    }
#endif
    digitalWrite(_ss, LOW);
    delay(2);     // or whatever the delay is for waking up the board
    spiwrite(PN532_SPI_DATAWRITE);

    checksum = PN532_PREAMBLE + PN532_PREAMBLE + PN532_STARTCODE2;
    spiwrite(PN532_PREAMBLE);
    spiwrite(PN532_PREAMBLE);
    spiwrite(PN532_STARTCODE2);

    spiwrite(cmdlen);
    uint8_t cmdlen_1=~cmdlen + 1;
    spiwrite(cmdlen_1);

    spiwrite(PN532_HOSTTOPN532);
    checksum += PN532_HOSTTOPN532;

#ifdef NDEF_DEBUG
    if (debug)
    {
        Serial.print(F(" ")); Serial.print(PN532_PREAMBLE, HEX);
        Serial.print(F(" ")); Serial.print(PN532_PREAMBLE, HEX);
        Serial.print(F(" ")); Serial.print(PN532_STARTCODE2, HEX);
        Serial.print(F(" ")); Serial.print(cmdlen, HEX);
        Serial.print(F(" ")); Serial.print(cmdlen_1, HEX);
        Serial.print(F(" ")); Serial.print(PN532_HOSTTOPN532, HEX);
    }
#endif

    for (uint8_t i=0; i<cmdlen-1; i++) {
        spiwrite(cmd[i]);
        checksum += cmd[i];
#ifdef NDEF_DEBUG
        if (debug)
        {
          Serial.print(F(" ")); Serial.print(cmd[i], HEX);
        }
#endif
    }
    uint8_t checksum_1=~checksum;
    spiwrite(checksum_1);
    spiwrite(PN532_POSTAMBLE);
    digitalWrite(_ss, HIGH);

#ifdef NDEF_DEBUG
    if (debug)
    {
      Serial.print(F(" 0x")); Serial.print(checksum_1, HEX);
      Serial.print(F(" 0x")); Serial.print(PN532_POSTAMBLE, HEX);
      Serial.println();
    }
#endif
}
/************** low level SPI */

void PN532::spiwrite(uint8_t c)
{
    int8_t i;
    digitalWrite(_clk, HIGH);

    for (i=0; i<8; i++)
    {
        digitalWrite(_clk, LOW);
        if (c & _BV(i))
        {
            digitalWrite(_mosi, HIGH);
        }
        else
        {
            digitalWrite(_mosi, LOW);
        }
        digitalWrite(_clk, HIGH);
    }
}

uint8_t PN532::spiread(void)
{
    int8_t i, x;
    x = 0;
    digitalWrite(_clk, HIGH);

    for (i=0; i<8; i++)
    {
        if (digitalRead(_miso))
        {
            x |= _BV(i);
        }
        digitalWrite(_clk, LOW);
        digitalWrite(_clk, HIGH);
    }
    return x;
}

boolean PN532_CMD_RESPONSE::verifyResponse(uint32_t cmdCode)
{
    return ( header[0] == 0x00 &&
             header[1] == 0xFF &&
            ((uint8_t)(len + len_chksum)) == 0x00 &&
            direction == 0xD5 &&
            (cmdCode + 1) == responseCode);
}

void PN532_CMD_RESPONSE::printResponse()
{
#ifdef NDEF_DEBUG
    Serial.print("response: ");

    uint8_t *ptr = (uint8_t *) this;
    for (uint8_t i = 0; i < sizeof(PN532_CMD_RESPONSE); i++)
    {
        if ((ptr + i) == &data_len)
        {
            continue;
        }
        Serial.print(ptr[i], HEX);
        Serial.print(F(" "));
    }

    for (uint8_t i = 0; i < data_len; ++i)
    {
        Serial.print(data[i], HEX);
        Serial.print(F(" "));
    }
    Serial.println();
#endif
}

/***** Mifare Classic Functions ******/

/**************************************************************************/
/*! 
      Indicates whether the specified block number is the first block
      in the sector (block 0 relative to the current sector)
*/
/**************************************************************************/
bool PN532::mifareclassic_IsFirstBlock (uint32_t uiBlock)
{
  // Test if we are in the small or big sectors
  if (uiBlock < 128)
    return ((uiBlock) % 4 == 0);
  else
    return ((uiBlock) % 16 == 0);
}

/**************************************************************************/
/*! 
      Indicates whether the specified block number is the sector trailer
*/
/**************************************************************************/
bool PN532::mifareclassic_IsTrailerBlock (uint32_t uiBlock)
{
  // Test if we are in the small or big sectors
  if (uiBlock < 128)
    return ((uiBlock + 1) % 4 == 0);
  else
    return ((uiBlock + 1) % 16 == 0);
}

/**************************************************************************/
/*! 
    Tries to authenticate a block of memory on a MIFARE card using the
    INDATAEXCHANGE command.  See section 7.3.8 of the PN532 User Manual
    for more information on sending MIFARE and other commands.

    @param  uid           Pointer to a byte array containing the card UID
    @param  uidLen        The length (in bytes) of the card's UID (Should
                          be 4 for MIFARE Classic)
    @param  blockNumber   The block number to authenticate.  (0..63 for
                          1KB cards, and 0..255 for 4KB cards).
    @param  keyNumber     Which key type to use during authentication
                          (0 = MIFARE_CMD_AUTH_A, 1 = MIFARE_CMD_AUTH_B)
    @param  keyData       Pointer to a byte array containing the 6 byte
                          key value
    
    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
uint8_t PN532::mifareclassic_AuthenticateBlock (uint8_t * uid, uint8_t uidLen, uint32_t blockNumber, uint8_t keyNumber, uint8_t * keyData)
{
  uint8_t len;
  uint8_t i;
  
  // Hang on to the key and uid data
  memcpy (_key, keyData, 6); 
  memcpy (_uid, uid, uidLen); 
  _uidLen = uidLen;  

 
  // Prepare the authentication command //
  pn532_packetbuffer[0] = PN532_INDATAEXCHANGE;   /* Data Exchange Header */
  pn532_packetbuffer[1] = 1;                              /* Max card numbers */
  pn532_packetbuffer[2] = (keyNumber) ? MIFARE_CMD_AUTH_B : MIFARE_CMD_AUTH_A;
  pn532_packetbuffer[3] = blockNumber;                    /* Block Number (1K = 0..63, 4K = 0..255 */
  memcpy (pn532_packetbuffer+4, _key, 6);
  for (i = 0; i < _uidLen; i++)
  {
    pn532_packetbuffer[10+i] = _uid[i];                /* 4 byte card ID */
  }

  if (! sendCommandCheckAck(pn532_packetbuffer, 10+_uidLen))
    {
		Serial.println("Error with ack");
		return 0;
	}
  // Read the response packet
  readspidata(pn532_packetbuffer, 12);
  // check if the response is valid and we are authenticated???
  // for an auth success it should be bytes 5-7: 0xD5 0x41 0x00
  // Mifare auth error is technically byte 7: 0x14 but anything other and 0x00 is not good
  if (pn532_packetbuffer[7] != 0x00)
  {
    Serial.print("Error with response ");
	Serial.println((int)pn532_packetbuffer[7], 16);
	return 0;
  }

  return 1;
}

/**************************************************************************/
/*! 
    Tries to read an entire 16-byte data block at the specified block
    address.

    @param  blockNumber   The block number to authenticate.  (0..63 for
                          1KB cards, and 0..255 for 4KB cards).
    @param  data          Pointer to the byte array that will hold the
                          retrieved data (if any)
    
    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
uint8_t PN532::mifareclassic_ReadDataBlock (uint8_t blockNumber, uint8_t * data)
{
  #ifdef MIFAREDEBUG
  Serial.print("Trying to read 16 bytes from block ");Serial.println(blockNumber);
  #endif
  
  /* Prepare the command */
  pn532_packetbuffer[0] = PN532_INDATAEXCHANGE;
  pn532_packetbuffer[1] = 1;                      /* Card number */
  pn532_packetbuffer[2] = MIFARE_CMD_READ;        /* Mifare Read command = 0x30 */
  pn532_packetbuffer[3] = blockNumber;            /* Block Number (0..63 for 1K, 0..255 for 4K) */

  /* Send the command */
  if (! sendCommandCheckAck(pn532_packetbuffer, 4))
  {
    #ifdef MIFAREDEBUG
    Serial.println("Failed to receive ACK for read command");
    #endif
    return 0;
  }

  /* Read the response packet */
  readspidata(pn532_packetbuffer, 26);

  /* If byte 8 isn't 0x00 we probably have an error */
  if (pn532_packetbuffer[7] != 0x00)
  {
    return 0;
  }
    
  /* Copy the 16 data bytes to the output buffer        */
  /* Block content starts at byte 9 of a valid response */
  memcpy (data, pn532_packetbuffer+8, 16);

  return 1;  
}

/**************************************************************************/
/*! 
    Tries to write an entire 16-byte data block at the specified block
    address.

    @param  blockNumber   The block number to authenticate.  (0..63 for
                          1KB cards, and 0..255 for 4KB cards).
    @param  data          The byte array that contains the data to write.
    
    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
uint8_t PN532::mifareclassic_WriteDataBlock (uint8_t blockNumber, uint8_t * data)
{
  #ifdef MIFAREDEBUG
  Serial.print("Trying to write 16 bytes to block ");Serial.println(blockNumber);
  #endif
  
  /* Prepare the first command */
  pn532_packetbuffer[0] = PN532_INDATAEXCHANGE;
  pn532_packetbuffer[1] = 1;                      /* Card number */
  pn532_packetbuffer[2] = MIFARE_CMD_WRITE;       /* Mifare Write command = 0xA0 */
  pn532_packetbuffer[3] = blockNumber;            /* Block Number (0..63 for 1K, 0..255 for 4K) */
  memcpy (pn532_packetbuffer+4, data, 16);          /* Data Payload */

  /* Send the command */
  if (! sendCommandCheckAck(pn532_packetbuffer, 20))
  {
    #ifdef MIFAREDEBUG
    Serial.println("Failed to receive ACK for write command");
    #endif
    return 0;
  }  
  delay(10);
  
  /* Read the response packet */
  readspidata(pn532_packetbuffer, 26);

  return 1;  
}

/**************************************************************************/
/*! 
    Formats a Mifare Classic card to store NDEF Records 
    
    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
uint8_t PN532::mifareclassic_FormatNDEF (void)
{
  uint8_t sectorbuffer1[16] = {0x14, 0x01, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1};
  uint8_t sectorbuffer2[16] = {0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1};
  uint8_t sectorbuffer3[16] = {0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0x78, 0x77, 0x88, 0xC1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

  // Write block 1 and 2 to the card
  if (!(mifareclassic_WriteDataBlock (1, sectorbuffer1)))
    return 0;
  if (!(mifareclassic_WriteDataBlock (2, sectorbuffer2)))
    return 0;
  // Write key A and access rights card
  if (!(mifareclassic_WriteDataBlock (3, sectorbuffer3)))
    return 0;

  // Seems that everything was OK (?!)
  return 1;
}

/**************************************************************************/
/*! 
    Writes an NDEF URI Record to the specified sector (1..15)
    
    Note that this function assumes that the Mifare Classic card is
    already formatted to work as an "NFC Forum Tag" and uses a MAD1
    file system.  You can use the NXP TagWriter app on Android to
    properly format cards for this.

    @param  sectorNumber  The sector that the URI record should be written
                          to (can be 1..15 for a 1K card)
    @param  uriIdentifier The uri identifier code (0 = none, 0x01 = 
                          "http://www.", etc.)
    @param  url           The uri text to write (max 38 characters).
    
    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
uint8_t PN532::mifareclassic_WriteNDEFURI (uint8_t sectorNumber, uint8_t uriIdentifier, const char * url)
{
  // Figure out how long the string is
  uint8_t len = strlen(url);
  
  // Make sure we're within a 1K limit for the sector number
  if ((sectorNumber < 1) || (sectorNumber > 15))
    return 0;
  
  // Make sure the URI payload is between 1 and 38 chars
  if ((len < 1) || (len > 38))
    return 0;
    
  // Setup the sector buffer (w/pre-formatted TLV wrapper and NDEF message)
  uint8_t sectorbuffer1[16] = {0x00, 0x00, 0x03, len+5, 0xD1, 0x01, len+1, 0x55, uriIdentifier, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t sectorbuffer2[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t sectorbuffer3[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t sectorbuffer4[16] = {0xD3, 0xF7, 0xD3, 0xF7, 0xD3, 0xF7, 0x7F, 0x07, 0x88, 0x40, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  if (len <= 6)
  {
    // Unlikely we'll get a url this short, but why not ...
    memcpy (sectorbuffer1+9, url, len);
    sectorbuffer1[len+9] = 0xFE;
  }
  else if (len == 7)
  {
    // 0xFE needs to be wrapped around to next block
    memcpy (sectorbuffer1+9, url, len);
    sectorbuffer2[0] = 0xFE;
  }
  else if ((len > 7) || (len <= 22))
  {
    // Url fits in two blocks
    memcpy (sectorbuffer1+9, url, 7);
    memcpy (sectorbuffer2, url+7, len-7);
    sectorbuffer2[len-7] = 0xFE;
  }
  else if (len == 23)
  {
    // 0xFE needs to be wrapped around to final block
    memcpy (sectorbuffer1+9, url, 7);
    memcpy (sectorbuffer2, url+7, len-7);
    sectorbuffer3[0] = 0xFE;
  }
  else
  {
    // Url fits in three blocks
    memcpy (sectorbuffer1+9, url, 7);
    memcpy (sectorbuffer2, url+7, 16);
    memcpy (sectorbuffer3, url+23, len-24);
    sectorbuffer3[len-22] = 0xFE;
  }
  
  // Now write all three blocks back to the card
  if (!(mifareclassic_WriteDataBlock (sectorNumber*4, sectorbuffer1)))
    return 0;
  if (!(mifareclassic_WriteDataBlock ((sectorNumber*4)+1, sectorbuffer2)))
    return 0;
  if (!(mifareclassic_WriteDataBlock ((sectorNumber*4)+2, sectorbuffer3)))
    return 0;
  if (!(mifareclassic_WriteDataBlock ((sectorNumber*4)+3, sectorbuffer4)))
    return 0;

  // Seems that everything was OK (?!)
  return 1;
}

/***** Mifare Ultralight Functions ******/

/**************************************************************************/
/*! 
    Tries to read an entire 4-byte page at the specified address.

    @param  page        The page number (0..63 in most cases)
    @param  buffer      Pointer to the byte array that will hold the
                        retrieved data (if any)
*/
/**************************************************************************/
uint8_t PN532::mifareultralight_ReadPage (uint8_t page, uint8_t * buffer)
{
  if (page >= 64)
  {
    #ifdef MIFAREDEBUG
    Serial.println("Page value out of range");
    #endif
    return 0;
  }

  #ifdef MIFAREDEBUG
    Serial.print("Reading page ");Serial.println(page);
  #endif

  /* Prepare the command */
  pn532_packetbuffer[0] = PN532_INDATAEXCHANGE;
  pn532_packetbuffer[1] = 1;                   /* Card number */
  pn532_packetbuffer[2] = MIFARE_CMD_READ;     /* Mifare Read command = 0x30 */
  pn532_packetbuffer[3] = page;                /* Page Number (0..63 in most cases) */

  /* Send the command */
  if (! sendCommandCheckAck(pn532_packetbuffer, 4))
  {
    return 0;
  }
  
  /* Read the response packet */
  readspidata(pn532_packetbuffer, 26);

  /* If byte 8 isn't 0x00 we probably have an error */
  if (pn532_packetbuffer[7] == 0x00)
  {
    /* Copy the 4 data bytes to the output buffer         */
    /* Block content starts at byte 9 of a valid response */
    /* Note that the command actually reads 16 byte or 4  */
    /* pages at a time ... we simply discard the last 12  */
    /* bytes                                              */
    memcpy (buffer, pn532_packetbuffer+8, 4);
  }
  else
  {
    return 0;
  }

  // Return OK signal
  return 1;
}

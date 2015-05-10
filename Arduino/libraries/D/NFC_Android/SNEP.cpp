
#include "SNEP.h"

SNEP::SNEP(NFCLinkLayer *linkLayer) :
    _linkLayer(linkLayer)
{
}

SNEP::~SNEP()
{
}

uint32_t SNEP::rxNDEFPayload(uint8_t *&data)
{
    uint32_t result = _linkLayer->openNPPServerLink();

    if(RESULT_OK(result)) //if connection is error-free
    {
       //Serial.println(F("CONNECTED."));
       result = _linkLayer->serverLinkRxData(data);
       if (RESULT_OK(result))
       {
	   // TODO: to check header of SNEP packet
            uint8_t *ptr = data;
            uint8_t version = *ptr;
            uint8_t require = *(ptr + 1);
            uint32_t length =  (ptr[2] << 24) + (ptr[3] << 16) + (ptr[4] << 8) + ptr[5];

            data = ptr + 6;
#ifdef NDEF_DEBUG
			Serial.print("NDEF message length: ");
            Serial.println(length);
#endif
			return length;
       }
    }
    return result;
}

uint32_t SNEP::pushPayload(uint8_t *NDEFMessage, uint32_t length)
{
   SNEP_MESSAGE *snepMessage = (SNEP_MESSAGE *) ALLOCATE_HEADER_SPACE(NDEFMessage, SNEP_MESSAGE_HDR_LEN);
   
   snepMessage->version = 0x10;
   snepMessage->action  = 0x02;
   snepMessage->length  = MODIFY_ENDIAN(length);
  
    uint32_t result = _linkLayer->openNPPClientLink();
    
    

    if(RESULT_OK(result)) //if connection is error-free
    {
        result =  _linkLayer->clientLinkTxData((uint8_t *)snepMessage, length + SNEP_MESSAGE_HDR_LEN);
    }
    
    

    return result;
}

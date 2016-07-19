/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @fn      SAP_close
 *
 * @brief   Tears down the port with the network processor
 *
 * @param   None.
 *
 * @return  uint8_t - SNP_SUCCESS if able to close
 */
uint8_t SAP_close(void)
{
  asyncCBNode_t *aNode = asyncCBListRoot;
  asyncCBNode_t *aTempNode = NULL;
  eventCBNode_t *eNode = eventCBListRoot;
  eventCBNode_t *eTempNode = NULL;
  serviceNode_t *sNode = serviceListRoot;
  serviceNode_t *sTempNode = NULL;

  // Clean up call back and service lists
  while(aNode != NULL)
  {
    aTempNode = (asyncCBNode_t *)aNode->next;
    SNP_free(aNode);
    aNode = aTempNode;
  }
  asyncCBListRoot = NULL;

  while(eNode != NULL)
  {
    eTempNode = (eventCBNode_t *)eNode->next;
    SNP_free(eNode);
    eNode = eTempNode;
  }
  eventCBListRoot = NULL;

  while(sNode != NULL)
  {
    sTempNode = (serviceNode_t *)sNode->next;
    SNP_free(sNode);
    sNode = sTempNode;
  }
  serviceListRoot = NULL;

  SNP_close();

  return NPITask_close();
}

/**
 * @brief       Write to a stack parameter on the SAP. Some responses will
 *              Return immediately, others will generate an event for which
 *              a callback must be registered with the correct event mask.
 *
 * @param       subsystemID - the subsystem ID: @ref SNP_RPC_PARAM_SUBSYSTEMS
 * @param       paramID     - the parameter within the subsystem to write
 * @param       len         - length of the data to write
 * @param       pData       - pointer to buffer of data to write
 *
 * @return      SNP_SUCCESS: the write completed successfully.<BR>
 *              SNP_FAILURE: stack parameter write failed.<BR>
 */
uint8_t SAP_setParam(uint8_t subsystemID, uint16_t paramID, uint16_t len,
                     uint8_t *pData)
{
  uint8_t status = SNP_SUCCESS;

  // Determine the subsystem.
  switch(subsystemID)
  {
    case SAP_PARAM_SECURITY:
      {
        snpSetSecParamReq_t lReq;

        switch(paramID)
        {
          case SAP_SECURITY_IOCAPS:
            lReq.paramId = SNP_GAPBOND_IO_CAPABILITIES;
            break;

          case SAP_SECURITY_BEHAVIOR:
            lReq.paramId = SNP_GAPBOND_PAIRING_MODE;
            break;

          case SAP_SECURITY_BONDING:
            lReq.paramId = SNP_GAPBOND_BONDING_ENABLED;
            break;

          case SAP_ERASE_ALL_BONDS:
            lReq.paramId = SNP_GAPBOND_ERASE_ALLBONDS;
            break;

          case SAP_ERASE_LRU_BOND:
            lReq.paramId = SNP_GAPBOND_LRU_BOND_REPLACEMENT;
            break;

          default:
            // Unknown command
            status = SNP_FAILURE;
            break;
        }

        if (status != SNP_FAILURE)
        {
          if (pData)
          {
            lReq.value = *pData;
          }
          else
          {
            lReq.value = NULL;
          }

          // Send set parameter request
          {
            snpSetSecParamRsp_t lRsp;

            SNP_RPC_setSecurityParam(&lReq, &lRsp);

            status = lRsp.status;
          }
        }
      }
      break;

    case SAP_PARAM_WHITELIST:
      {
        snpSetWhiteListReq_t lReq;

        lReq.useWhiteList = (*pData == SAP_WHITELIST_ENABLE) ?
                                SNP_FILTER_POLICY_WHITE : SNP_FILTER_POLICY_ALL;

        status = SNP_RPC_setWhiteListParam(&lReq);
      }
      break;

    default:
      // Unknown command
      status = SNP_FAILURE;
      break;
  }

  return status;
}

/**
 * @brief       Send a Security Request.  For a Peripheral device, this is a
 *              Slave Security Request not a Pairing Request.
 *
 * @param       None
 *
 * @return      SNP_SUCCESS: security was requested.<BR>
 *              SNP_RPC_FAILRUE: security request failed.<BR>
 */
uint8_t SAP_sendSecurityRequest(void)
{
  return SNP_RPC_sendSecurityRequest();
}

/**
 * @brief       Send authentication data to SNP.
 *
 * @param       authData - authentication data.  This is the passkey used for
 *                         for the passkey entry protocol.
 *                         For numeric comparisons, this is TRUE if equivalent
 *                         or FALSE if not.
 *
 * @return      SNP_SUCCESS: authentication data sent.<BR>
 *              SNP_RPC_FAILRUE: authentication data failed to be sent.<BR>
 */
uint8_t SAP_setAuthenticationRsp(uint32_t authData)
{
  snpSetAuthDataReq_t pReq;

  pReq.authData = authData;
  return SNP_RPC_setAuthenticationData(&pReq);
}

#include "utils.h"
void getCommand(char* pSource, uint16_t Lsource, char* pDest)
{
    uint8_t ptr = 0;
    while((*(pSource + ptr)) <= ' ') ptr++; // trim the data to find first non-blank char
    // then we should find next blank char
    // command support A~Z, a~z, and underline _
    for(uint8_t i = 0; i < MAX_COMMAND_WORD_LENGTH + 1; i++)
    {
        if((*(pSource + ptr + i) >= 'A' && *(pSource + ptr + i) <= 'Z') || (*(pSource + ptr + i) >= 'a' && *(pSource + ptr + i) <= 'z') || (*(pSource + ptr + i) == '_'))
        {
            *(pDest + i) = *(pSource + ptr + i);
        }
        else
        {
            *(pDest + i) = '\0';
            return;
        }
    }
}
// 支持将字节数组按 split 字符分割为浮点参数数组
void splitData_f(char* pSource, uint16_t Lsource, float* pDest, uint8_t* Ldest, uint8_t split) 
{
    uint8_t temp = 0, point = 0, negative = 0;
    uint16_t i = 0, numCount = 0;
    *(Ldest) = 0;
    // for(i = 0; i < maxLen; i++) *(pDest+i) = 0; // flush the dest buffer, maxLen represents maximum length of pDest (prevent overflow)
    for(i = 0; i < Lsource && *(Ldest) <= Lsource; i++)
    {
        temp = *(pSource + i);
        if(temp != split)
        {
            if(numCount == 0 && *(pDest + *(Ldest)) != 0) *(pDest + *(Ldest)) = 0.0f; // flush the used pDest buffer
            if(temp == '-')
            {
                if(*(pDest + *(Ldest)) != 0) *(pDest + *(Ldest)) = 0.0f; // error: '-' after numbers, ignoring forward nums
                negative = 1;
            }
            else if(temp == '.')
            { // point
                if(point) point = 0; // error: multiple points in one continous number
                else point = 1;
            }
            else if(temp >= '0' && temp <= '9')
            {
                if(point)
                {
                    *(pDest + *(Ldest)) += (float)((temp - '0') / (powf(10, point++)));
                }
                else
                {
                    *(pDest + *(Ldest)) *= 10.0f;
                    *(pDest + *(Ldest)) += (float)(temp - '0');
                }
                numCount++;
            }
        }
        if(temp == split || i == Lsource - 1)
        {
            if(*(pDest + *(Ldest)) != 0 || (*(pDest + *(Ldest)) == 0 && numCount > 0))
            { // (pDest[Ldest] != 0 || (pDest[Ldest] == 0 && numCount >0) && temp == split, check negative and move Ldest pointer back
                if(negative) *(pDest + *(Ldest)) = (float)(-1.0f * (*(pDest + *(Ldest))));
                *(Ldest) += 1;
            }
            negative = point = numCount = 0;
        }
    }
}

uint8_t trimData(char* pSource)
{
    uint8_t result = 0;
    while((*(pSource + result)) <= ' ') result++;
    return result;
}
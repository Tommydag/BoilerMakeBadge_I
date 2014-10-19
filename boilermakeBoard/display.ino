
#define AR_SIZE( a ) sizeof( a ) / sizeof( a[0] )


unsigned char en_char1[]=" Stockoholic: ";
unsigned char en_char2[]=" Big Data, Stocks ";
unsigned char en_char3[]=" Bloomberg ";


void initDisplay()
{
  LCDA.initDriverPin(4,12,13); //INIT SPI Interface
  LCDA.Initialise(); // INIT SCREEN
  delay(100);
  LCDA.CLEAR();
  delay(100);
  LCDA.DisplayString(0,0,en_char1,16); 
   LCDA.DisplayString(1,0,en_char2,16); 
    LCDA.DisplayString(2,0,en_char3,16); 
  delay(30);
}


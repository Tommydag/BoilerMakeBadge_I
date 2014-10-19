

#define AR_SIZE( a ) sizeof( a ) / sizeof( a[0] )


unsigned char en_char1[]=" BoilerBake 420 ";

void initDisplay()
{
  LCDA.initDriverPin(2,3,4); //INIT SPI Interface
  LCDA.Initialise(); // INIT SCREEN
  delay(100);
  LCDA.CLEAR();
  delay(100);
  LCDA.DisplayString(0,0,en_char1,16); 
  delay(30);
}
 

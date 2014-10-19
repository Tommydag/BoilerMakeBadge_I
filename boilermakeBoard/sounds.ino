

int noteDurations[] = {
    4, 8, 8, 4,4,4,4,4   };
    
int noteDurations2[] = {
    8,8,8,8,
    8,8,8,8,
    8,8,8,8,
    8,8,8,8,
    12,4,12,4};



  int melody2[] = {
    NOTE_E4, NOTE_E4,0,NOTE_E4,
    0, NOTE_C4, NOTE_E4,0,
    NOTE_G4,0,0,0,
    NOTE_G3,0,0,0,
    NOTE_B6, NOTE_E6, NOTE_B6, NOTE_E6};


  int melody[] = {
    NOTE_C4, NOTE_G3,NOTE_G3, NOTE_A3, NOTE_G3,0, NOTE_B3, NOTE_C4  };

void playTone(){
   

  for (int thisNote = 0; thisNote < 22; thisNote++) {

    // to calculate the note duration, take one second 
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration2 = 1000/noteDurations2[thisNote];
    tone(5, melody2[thisNote],noteDuration2);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration2 * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(5);
  }
}

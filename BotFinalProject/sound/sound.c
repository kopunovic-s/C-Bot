#include "sound.h"
#include "../movement/open_interface.h"

void sound_init()
{                                                                        // Ensure that this initialization is only used after oi_alloc and oi_init
    unsigned char notes[] = {0, 67, 67, 67, 64, 0, 65, 65, 65, 62};      // MIDI note numbering, see chart in Open Interface documentation
    unsigned char lengths[] = {18, 18, 18, 18, 72, 18, 18, 18, 18, 108}; // Lengths in 64ths of a second

    oi_loadSong(0, 10, notes, lengths); // Loads the above notes and lengths into a 10 note song indexed at 0

    unsigned char notesB[] = {60, 62, 65, 62, 69, 69, 67, 60, 62, 65, 62, 67, 67, 65, 64, 62};
    unsigned char lengthsB[] = {8, 8, 8, 8, 16, 24, 48, 8, 8, 8, 8, 16, 24, 48, 8, 16};

    oi_loadSong(1, 16, notesB, lengthsB); // Loads the above notes and lengths into a 16 note song indexed at 1
}

void sound_bomb()
{ // Plays song 0, use at bomb
    oi_play_song(0);
}

void sound_home()
{ // Plays song 1, use when done
    oi_play_song(1);
}

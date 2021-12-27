
#ifndef DISPLAY_H_
#define DISPLAY_H_

void setup_display();
void sleep_display();
void wakeup_display();
void stop_display();

void show_display(String header, int wait = 0);
void show_display(String header, String line1, int wait = 0);
void show_display(String header, String line1, String line2, int wait = 0);
void show_display(String header, String line1, String line2, String line3, int wait = 0);
void show_display(String header, String line1, String line2, String line3, String line4, int wait = 0);
void show_display(String header, bool isTxing, String line1, String line2, String line3, String line4, String line5, int wait = 0);
void displayLogoAndMsg(String msg, uint32_t wait_ms);
void show_display_menu(String header, String line1, String line2, String line3, String line4, int wait = 0);

#endif

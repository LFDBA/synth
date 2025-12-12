#include <ncurses.h>
#include <vector>
#include <string>

int main() {
    // Initialize ncurses
    initscr();
    noecho();
    cbreak();
    keypad(stdscr, TRUE);
    curs_set(0); // hide cursor

    // Colors
    if (has_colors()) {
        start_color();
        init_pair(1, COLOR_BLACK, COLOR_CYAN);   // Highlighted option
        init_pair(2, COLOR_WHITE, COLOR_BLACK);  // Normal option
    }

    std::vector<std::string> menuItems = {"TONE", "VOICE", "ADSR", "REVERB"};
    int choice = 0;
    int key;

    while (true) {
        clear();
        int row, col;
        getmaxyx(stdscr, row, col);

        int menuWidth = 20;
        int startY = (row - menuItems.size()*3) / 2; // center vertically
        int startX = (col - menuWidth) / 2;          // center horizontally

        for (size_t i = 0; i < menuItems.size(); i++) {
            int y = startY + i*3;

            // Draw a "box" around the option
            for (int bx = 0; bx < menuWidth; bx++) {
                mvprintw(y, startX + bx, "-");
                mvprintw(y + 2, startX + bx, "-");
            }
            mvprintw(y + 1, startX, "|");
            mvprintw(y + 1, startX + menuWidth - 1, "|");

            // Highlight selected option
            if (i == choice)
                attron(COLOR_PAIR(1) | A_BOLD);
            else
                attron(COLOR_PAIR(2));

            mvprintw(y + 1, startX + 2, "%s", menuItems[i].c_str());

            attroff(COLOR_PAIR(1) | COLOR_PAIR(2) | A_BOLD);
        }

        refresh();

        key = getch();
        if (key == KEY_UP) {
            if (choice > 0) choice--;
        } else if (key == KEY_DOWN) {
            if (choice < menuItems.size() - 1) choice++;
        } else if (key == '\n') {
            // Enter pressed
            break;
        }
    }

    // Clear and print selection
    clear();
    mvprintw(LINES / 2, (COLS - 20)/2, "Selected: %s", menuItems[choice].c_str());
    refresh();
    getch();

    endwin(); // end ncurses
    return 0;
}

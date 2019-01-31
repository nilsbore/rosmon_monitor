#include <iostream>
#include <vector>
#include <fort.hpp>
#include <yaml-cpp/yaml.h>

int main()
{
    fort::table table;
    // Fill table with data
    table << fort::header
        << "Rank" << "Title" << "Year" << "Rating" << fort::endr
        << "1" << "The Shawshank Redemption" << "1994" << "9.5" << fort::endr
        << "2" << "12 Angry Men" << "1957" << "8.8" << fort::endr
        << "3" << "It's a Wonderful Life" << "1946" << "8.6" << fort::endr
        << fort::separator
        << "4" << "2001: A Space Odyssey" << "1968" << "8.5" << fort::endr
        << "5" << "Blade Runner" << "1982" << "8.1" << fort::endr
        << fort::endr;

    table[0][1].set_cell_bg_color(fort::color::red);
    table[0][0].set_cell_min_width(20);
    table[0][0].set_cell_text_align(fort::text_align::left);
    table[2].set_cell_row_type(fort::row_type::header);

    std::cout << table.to_string() << std::endl;

    return 0;
}

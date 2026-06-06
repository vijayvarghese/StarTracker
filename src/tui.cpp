#include "tui.hpp"
#include "globals.h"

#include <ftxui/component/component.hpp>
#include <ftxui/component/event.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <ftxui/dom/elements.hpp>
#include <ftxui/screen/color.hpp>
#include "ftxui/dom/elements.hpp"

using namespace ftxui;

TuiThread::TuiThread(const std::string& name, const std::string& version)
    : name_(name), version_(version) {}

void TuiThread::start() {
    thread_ = std::thread(&TuiThread::run, this);
}

void TuiThread::join() {
    if (thread_.joinable()) thread_.join();
}


//Things need to integrate 
bool img_on = false;
int dbg_level = 0;
int log_idx = 0;
const char* log_labels[] = { "INFO", "DEBUG", "WARN", "ERR", "ALL" };
auto start_time = std::chrono::steady_clock::now();
bool done = false;
int log_scroll = 0;
std::vector<std::string> logs = {
    "[INFO] Started",
    "[INFO] Loading catalog...",
    "[WARN] Low confidence",
    "[ERR]  Lost track",
};  // dummy data for now, real logs come later





void TuiThread::run() {
    auto screen = ScreenInteractive::Fullscreen();

    auto dbg_color = [](int lvl) -> Color {
        switch(lvl) {
            case 1: return Color::Green;
            case 2: return Color::RGB(144, 238, 0);
            case 3: return Color::Yellow;
            case 4: return Color::RGB(255, 140, 0);
            case 5: return Color::Red;
            default: return Color::GrayDark;
        }
    };

    auto log_color = [](int idx) -> Color {
    switch(idx) {
        case 0: return Color::Cyan;
        case 1: return Color::Green;
        case 2: return Color::Yellow;
        case 3: return Color::Red;
        case 4: return Color::Magenta;
        default: return Color::White;
    }
};

auto format_runtime = [&]() -> std::string {
    using namespace std::chrono;
    auto secs = duration_cast<seconds>(steady_clock::now() - start_time).count();
    int h = secs / 3600;
    int m = (secs % 3600) / 60;
    int s = secs % 60;
    char buf[16];
    snprintf(buf, sizeof(buf), "%02d:%02d:%02d", h, m, s);
    return buf;
};



    auto ui = Renderer([&] {

        // ── title ─────────────────────────────────────────────────────────
        auto title = text(" " + name_ + "  v" + version_ + " ")
                        | bold | color(Color::RGB(100, 200, 255)) | flex;

        // ── static cells ──────────────────────────────────────────────────
        auto img_cell = hbox({
            text(" IMG: ") | color(Color::GrayDark),
            text(img_on ? "ON " : "OFF ") | bold
                | color(img_on ? Color::Green : Color::GrayDark),
        }) | border;

        auto dbg_cell = hbox({
            text(" DBG: ") | color(Color::GrayDark),
            text(dbg_level == 0 ? "OFF " : std::to_string(dbg_level) + " ")
                | bold | color(dbg_color(dbg_level)),
        }) | border;

        auto log_cell = hbox({
            text(" LOG: ") | color(Color::GrayDark),
            text(std::string(log_labels[log_idx]) + " ")
                | bold | color(log_color(log_idx)),
        }) | border;

        auto q_cell = text("  Q  ") | border;

        // ── Second Row Begins Here ──────────────────────────────────────────────────
        auto threads_cell = vbox({
            text(" Threads") | bold | color(Color::RGB(180, 180, 255)),
            text("  Reader     33.0 Hz") | color(Color::Green),
            text("  Processor   1.0 Hz") | color(Color::Green),
        }) | border | flex;

        auto lookup_cell = vbox({
            text(" Info") | bold | color(Color::RGB(180, 180, 255)),
            text("  Runtime    : " + format_runtime()) | color(Color::Cyan),
            text("  Confidence : 0.0%")     | color(Color::White),
            text("  Lookup     : hipparcos_mag3.bin") | color(Color::White),
            text("  Config     : config.json")        | color(Color::White),
        }) | border | flex;

        auto config_cell = vbox({
            text(" Centroids") | bold | color(Color::RGB(180, 180, 255)),
            text("  (none yet)") | color(Color::GrayDark),
        }) | border | flex;



        // ── Second Row Begins Here ──────────────────────────────────────────────────    
        Elements log_lines;
        int total = logs.size();
        int visible = total - log_scroll;
        int start = std::max(0, visible);

        for (int i = start; i < total; i++) {
    const std::string& line = logs[i];
    
    Color c = Color::GrayLight;
    if      (line.find("[ERR]")   != std::string::npos) c = Color::Red;
    else if (line.find("[WARN]")  != std::string::npos) c = Color::Yellow;
    else if (line.find("[DBG]")   != std::string::npos) c = Color::Green;
    else if (line.find("[INFO]")  != std::string::npos) c = Color::Cyan;

    log_lines.push_back(text("  " + line) | color(c));
}
        auto log_row = vbox({
            text(" Logs") | bold | color(Color::RGB(180, 180, 255)),
            vbox(log_lines),
        }) | border | flex;        


        // ── assemble header row ───────────────────────────────────────────
        return vbox({
        hbox({ title, img_cell, dbg_cell, log_cell, q_cell }),
        hbox({ threads_cell, lookup_cell, config_cell }),
        log_row | flex,
        });
    });

    auto root = CatchEvent(ui, [&](Event e) {
        if (e == Event::Character('q') || e == Event::Character('Q')) {
            screen.ExitLoopClosure()();
            ST::running.store(false);
            return true;
        }
        if (e == Event::Character('i') || e == Event::Character('I')) {
            img_on = !img_on;
            return true;
        }
        if (e == Event::Character('d')) {
            dbg_level = (dbg_level + 1) % 6;
            ST::log::log_level = dbg_level;
            return true;
        }
        if (e == Event::Character('D')) {
            dbg_level = 0;
            return true;
        }

        if (e == Event::Character('l') || e == Event::Character('L')) {
            log_idx = (log_idx + 1) % 5;
            return true;
        }        

        if (e == Event::ArrowUp) {
            log_scroll++;
            return true;
        }
        if (e == Event::ArrowDown) {
            log_scroll = std::max(0, log_scroll - 1);
            return true;
        }

        return false;
    });

    std::thread refresh([&] {
        while (!done) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            screen.PostEvent(Event::Custom);
        }
    });

    screen.Loop(root);
    done = true;
    refresh.join();
}
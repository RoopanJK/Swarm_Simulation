#include <unordered_map>
#include <vector>

enum Axis
{
    cx,
    cy,
    xi,
    yi,
    ic //Induct Closer
};

class Goal
{
public:
    Axis axis;
    int point;
    int checkPoint;
    Goal()
    {
        axis = cx;
        point = 0;
        checkPoint = 0;
    }
    Goal(Axis _axis, float _point, int _checkPoint = 0)
    {
        axis = _axis;
        point = _point;
        checkPoint = _checkPoint;
    }
};

const double xInduct[1] = {0.9381};
const double yInduct[2] = {0.3528, 0.6534};
const double cxPoint[7] = {0.8440, 0.7202, 0.6057, 0.5009, 0.3772, 0.2633, 0.1592};
const double cyPoint[7] = {0.6506, 0.7327, 0.6188, 0.4980, 0.3824, 0.2877, 0.1605};

std::vector<Goal> one_one_waypoint = {Goal(cx, 1), Goal(cy, 5), Goal(cx, 3), Goal(cy, 2), Goal(ic, 1)};
std::vector<Goal> one_two_waypoint = {Goal(cx, 1), Goal(cy, 5), Goal(cx, 3), Goal(cy, 2), Goal(ic, -1)};
std::vector<Goal> one_three_waypoint = {Goal(cx, 1), Goal(cy, 5), Goal(cx, 5), Goal(cy, 2), Goal(ic, -1)};
std::vector<Goal> one_four_waypoint = {Goal(cx, 1), Goal(cy, 5), Goal(cx, 3), Goal(cy, 4), Goal(ic, 1)};
std::vector<Goal> one_five_waypoint = {Goal(cx, 1), Goal(cy, 5), Goal(cx, 3), Goal(cy, 4), Goal(ic, -1)};
std::vector<Goal> one_six_waypoint = {Goal(cx, 1), Goal(cy, 5), Goal(cx, 5), Goal(cy, 4), Goal(ic, -1)};
std::vector<Goal> one_seven_waypoint = {Goal(cx, 1), Goal(cy, 5), Goal(cx, 3), Goal(cy, 6), Goal(ic, 1)};
std::vector<Goal> one_eight_waypoint = {Goal(cx, 1), Goal(cy, 5), Goal(cx, 3), Goal(cy, 6), Goal(ic, -1)};
std::vector<Goal> one_nine_waypoint = {Goal(cx, 1), Goal(cy, 5), Goal(cx, 5), Goal(cy, 6), Goal(ic, -1)};


std::vector<Goal> two_one_waypoint = {Goal(cx, 1), Goal(cy, 3), Goal(cx, 3), Goal(cy, 2), Goal(ic, 1)};
std::vector<Goal> two_two_waypoint = {Goal(cx, 1), Goal(cy, 3), Goal(cx, 3), Goal(cy, 2), Goal(ic, -1)};
std::vector<Goal> two_three_waypoint = {Goal(cx, 1), Goal(cy, 3), Goal(cx, 5), Goal(cy, 2), Goal(ic, -1)};
std::vector<Goal> two_four_waypoint = {Goal(cx, 1), Goal(cy, 3), Goal(cx, 3), Goal(cy, 4), Goal(ic, 1)};
std::vector<Goal> two_five_waypoint = {Goal(cx, 1), Goal(cy, 3), Goal(cx, 3), Goal(cy, 4), Goal(ic, -1)};
std::vector<Goal> two_six_waypoint = {Goal(cx, 1), Goal(cy, 3), Goal(cx, 5), Goal(cy, 4), Goal(ic, -1)};
std::vector<Goal> two_seven_waypoint = {Goal(cx, 1), Goal(cy, 3), Goal(cx, 3), Goal(cy, 6), Goal(ic, 1)};
std::vector<Goal> two_eight_waypoint = {Goal(cx, 1), Goal(cy, 3), Goal(cx, 3), Goal(cy, 6), Goal(ic, -1)};
std::vector<Goal> two_nine_waypoint = {Goal(cx, 1), Goal(cy, 3), Goal(cx, 5), Goal(cy, 6), Goal(ic, -1)};

// Return from Goal Points

std::vector<Goal> r_one_one_waypoint = {Goal(cx, 3), Goal(cy, 7), Goal(cx, 1), Goal(yi, 1), Goal(xi, 1)};
std::vector<Goal> r_one_two_waypoint = {Goal(cx, 3), Goal(cy, 7), Goal(cx, 1), Goal(yi, 1), Goal(xi, 1)};
std::vector<Goal> r_one_three_waypoint = {Goal(cx, 5), Goal(cy, 7), Goal(cx, 1), Goal(yi, 1), Goal(xi, 1)};
std::vector<Goal> r_one_four_waypoint = {Goal(cx, 3), Goal(cy, 7), Goal(cx, 1), Goal(yi, 1), Goal(xi, 1)};
std::vector<Goal> r_one_five_waypoint = {Goal(cx, 3), Goal(cy, 7), Goal(cx, 1), Goal(yi, 1), Goal(xi, 1)};
std::vector<Goal> r_one_six_waypoint = {Goal(cx, 5), Goal(cy, 7), Goal(cx, 1), Goal(yi, 1), Goal(xi, 1)};
std::vector<Goal> r_one_seven_waypoint = {Goal(cx, 3), Goal(cy, 7), Goal(cx, 1), Goal(yi, 1), Goal(xi, 1)};
std::vector<Goal> r_one_eight_waypoint = {Goal(cx, 3), Goal(cy, 7), Goal(cx, 1), Goal(yi, 1), Goal(xi, 1)};
std::vector<Goal> r_one_nine_waypoint = {Goal(cx, 5), Goal(cy, 7), Goal(cx, 1), Goal(yi, 1), Goal(xi, 1)};


std::vector<Goal> r_two_one_waypoint = {Goal(cx, 3), Goal(cy, 1), Goal(cx, 1), Goal(yi, 1), Goal(xi, 2)};
std::vector<Goal> r_two_two_waypoint = {Goal(cx, 3), Goal(cy, 1), Goal(cx, 1), Goal(yi, 1), Goal(xi, 2)};
std::vector<Goal> r_two_three_waypoint = {Goal(cx, 5), Goal(cy, 1), Goal(cx, 1), Goal(yi, 1), Goal(xi, 2)};
std::vector<Goal> r_two_four_waypoint = {Goal(cx, 3), Goal(cy, 1), Goal(cx, 1), Goal(yi, 1), Goal(xi, 2)};
std::vector<Goal> r_two_five_waypoint = {Goal(cx, 3), Goal(cy, 1), Goal(cx, 1), Goal(yi, 1), Goal(xi, 2)};
std::vector<Goal> r_two_six_waypoint = {Goal(cx, 5), Goal(cy, 1), Goal(cx, 1), Goal(yi, 1), Goal(xi, 2)};
std::vector<Goal> r_two_seven_waypoint = {Goal(cx, 3), Goal(cy, 1), Goal(cx, 1), Goal(yi, 1), Goal(xi, 2)};
std::vector<Goal> r_two_eight_waypoint = {Goal(cx, 3), Goal(cy, 1), Goal(cx, 1), Goal(yi, 1), Goal(xi, 2)};
std::vector<Goal> r_two_nine_waypoint = {Goal(cx, 5), Goal(cy, 1), Goal(cx, 1), Goal(yi, 1), Goal(xi, 2)};


// Unordered Table (Key Value pairs)

std::unordered_map<std::string, std::vector<Goal>> umap = {
    {"1_1", one_one_waypoint},
    {"1_2", one_two_waypoint},
    {"1_3", one_three_waypoint},
    {"1_4", one_four_waypoint},
    {"1_5", one_five_waypoint},
    {"1_6", one_six_waypoint},
    {"1_7", one_seven_waypoint},
    {"1_8", one_eight_waypoint},
    {"1_9", one_nine_waypoint},
    {"2_1", two_one_waypoint},
    {"2_2", two_two_waypoint},
    {"2_3", two_three_waypoint},
    {"2_4", two_four_waypoint},
    {"2_5", two_five_waypoint},
    {"2_6", two_six_waypoint},
    {"2_7", two_seven_waypoint},
    {"2_8", two_eight_waypoint},
    {"2_9", two_nine_waypoint},
    {"r_1_1", r_one_one_waypoint},
    {"r_1_2", r_one_two_waypoint},
    {"r_1_3", r_one_three_waypoint},
    {"r_1_4", r_one_four_waypoint},
    {"r_1_5", r_one_five_waypoint},
    {"r_1_6", r_one_six_waypoint},
    {"r_1_7", r_one_seven_waypoint},
    {"r_1_8", r_one_eight_waypoint},
    {"r_1_9", r_one_nine_waypoint},
    {"r_2_1", r_two_one_waypoint},
    {"r_2_2", r_two_two_waypoint},
    {"r_2_3", r_two_three_waypoint},
    {"r_2_4", r_two_four_waypoint},
    {"r_2_5", r_two_five_waypoint},
    {"r_2_6", r_two_six_waypoint},
    {"r_2_7", r_two_seven_waypoint},
    {"r_2_8", r_two_eight_waypoint},
    {"r_2_9", r_two_nine_waypoint},
};






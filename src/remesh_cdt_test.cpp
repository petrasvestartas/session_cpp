#include "mini_test.h"
#include "remesh_cdt.h"
#include "session_config.h"
#include "tolerance.h"

#include <cmath>

using namespace session_cpp::mini_test;

namespace session_cpp {


    MINI_TEST("RemeshCDT", "Triangulate") {
        // uncomment #include "mesh.h"
        // uncomment #include "polyline.h"
        // uncomment #include "remesh_cdt.h"


        Polyline border({
            Point(0, 0, 0),
            Point(4, 0, 0),
            Point(4, 4, 0),
            Point(0, 4, 0)
        });
        Polyline hole({
            Point(1, 1, 0),
            Point(1, 3, 0),
            Point(3, 3, 0),
            Point(3, 1, 0),
        });

        auto tris = RemeshCDT::triangulate({border, hole});

        // collect points
        std::vector<Point> flat;
        for (const auto& p : border.get_points())
            flat.push_back(p);
        for (const auto& p : hole.get_points())
            flat.push_back(p);

        Mesh m;

        // add vertices and collect indices
        std::vector<size_t> vkeys;
        for (const auto& p : flat)
            vkeys.push_back(m.add_vertex(p));

        // iterate faces
        for (const auto& t : tris)
            m.add_face({vkeys[t[0]], vkeys[t[1]], vkeys[t[2]]});

        MINI_CHECK(m.is_valid());
    }

    MINI_TEST("RemeshCDT", "Triangle") {
        // uncomment #include "mesh.h"
        // uncomment #include "polyline.h"
        // uncomment #include "remesh_cdt.h"

        Polyline pl({
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(0, 1, 0),
        });
        Mesh m = RemeshCDT::from_polylines({pl});

        MINI_CHECK(m.is_valid());
    }

    MINI_TEST("RemeshCDT", "Rectangle") {
        // uncomment #include "mesh.h"
        // uncomment #include "polyline.h"
        // uncomment #include "remesh_cdt.h"

        Polyline pl({
            Point(3, 0, 0),
            Point(5, 0, 0),
            Point(5, 2, 0),
            Point(3, 2, 0),
        });
        Mesh m = RemeshCDT::from_polylines({pl});

        MINI_CHECK(m.is_valid());
    }

    MINI_TEST("RemeshCDT", "L-shape") {
        // uncomment #include "mesh.h"
        // uncomment #include "polyline.h"
        // uncomment #include "remesh_cdt.h"

        Polyline pl({
            Point(7, 0, 0),
            Point(10, 0, 0),
            Point(10, 1, 0),
            Point(8, 1, 0),
            Point(8, 3, 0),
            Point(7, 3, 0),
        });
        Mesh m = RemeshCDT::from_polylines({pl});

        MINI_CHECK(m.is_valid());
    }

    MINI_TEST("RemeshCDT", "U-shape") {
    // uncomment #include "mesh.h"
    // uncomment #include "polyline.h"
    // uncomment #include "remesh_cdt.h"

    Polyline pl({
        Point(25, 0, 0),
        Point(31, 0, 0),
        Point(31, 4, 0),
        Point(29, 4, 0),
        Point(29, 2, 0),
        Point(27, 2, 0),
        Point(27, 4, 0),
        Point(25, 4, 0),
    });
    Mesh m = RemeshCDT::from_polylines({pl});

    MINI_CHECK(m.is_valid());
    }

    MINI_TEST("RemeshCDT", "Octagon") {
        // uncomment #include "mesh.h"
        // uncomment #include "polyline.h"
        // uncomment #include "remesh_cdt.h"

        Polyline pl = Polyline::from_sides(8, 1.5);
        pl += Vector(14, 1.5, 0);
        Mesh m = RemeshCDT::from_polylines({pl});

        MINI_CHECK(m.is_valid());
    }

    MINI_TEST("RemeshCDT", "Rectangle with rectangle hole") {
        // uncomment #include "mesh.h"
        // uncomment #include "polyline.h"
        // uncomment #include "remesh_cdt.h"

        Polyline border({
            Point(0, 0, 0),
            Point(4, 0, 0),
            Point(4, 4, 0),
            Point(0, 4, 0),
        });
        Polyline hole({
            Point(1, 1, 0),
            Point(1, 3, 0),
            Point(3, 3, 0),
            Point(3, 1, 0),
        });
        Mesh m = RemeshCDT::from_polylines({border, hole});

        MINI_CHECK(m.is_valid());
    }

    MINI_TEST("RemeshCDT", "Duplicate vertices") {
        // uncomment #include "mesh.h"
        // uncomment #include "polyline.h"
        // uncomment #include "remesh_cdt.h"

        Polyline pl({
            Point(33, 0, 0),
            Point(36, 0, 0),
            Point(37, 2, 0),
            Point(35, 3, 0),
            Point(33, 2, 0),
            Point(33, 0, 0),
        });
        Mesh m = RemeshCDT::from_polylines({pl});

        MINI_CHECK(m.is_valid());
    }

    MINI_TEST("RemeshCDT", "Tilted rectangle with rectangle hole") {
        // uncomment #include "mesh.h"
        // uncomment #include "polyline.h"
        // uncomment #include "remesh_cdt.h"

        Mesh m = RemeshCDT::from_polylines({
            Polyline({
                Point(55, 0, 0),
                Point(62, 0, 0),
                Point(62, 4, 2),
                Point(55, 4, 2),
            }),
            Polyline({
                Point(56, 1, 0.5),
                Point(61, 1, 0.5),
                Point(61, 3, 1.5),
                Point(56, 3, 1.5),
            }),
        }, false, false);

        MINI_CHECK(m.is_valid());
    }

    MINI_TEST("RemeshCDT", "Irregular tilted polyline.") {
        // uncomment #include "mesh.h"
        // uncomment #include "polyline.h"
        // uncomment #include "remesh_cdt.h"

        std::vector<Point> border = {
            Point(125.390575, 14.236865, -16.468853),
            Point(115.72382, 17.091624, -3.285212),
            Point(121.789447, 21.876306, 18.811062),
            Point(151.252023, 21.746582, 18.211977),
            Point(142.916699, 15.485629, -10.701904),
            Point(135.657716, 19.277143, 6.807792),
            Point(142.118715, 19.142866, 6.187686),
            Point(141.57586, 17.857892, 0.253509),
            Point(140.11301, 18.526023, 3.339026),
            Point(141.21524, 18.751613, 4.38083),
            Point(141.292725, 18.989286, 5.478431),
            Point(139.566886, 19.055145, 5.78258),
            Point(138.763256, 18.580689, 3.59148),
            Point(139.310849, 18.055272, 1.165039),
            Point(140.386846, 17.829745, 0.123525),
            Point(140.874682, 17.186517, -2.846986),
            Point(142.977469, 16.726916, -4.969483),
            Point(144.463527, 17.732299, -0.326495),
            Point(144.003555, 19.468828, 7.693018),
            Point(133.951268, 20.010289, 10.193557),
            Point(143.92422, 20.548103, 12.677251),
            Point(134.28842, 20.656763, 13.179053),
            Point(141.475986, 21.027044, 14.889059),
            Point(148.395425, 21.092848, 15.192952),
            Point(146.816239, 20.209864, 11.115218),
            Point(140.129758, 20.039236, 10.327236),
            Point(145.879946, 19.943669, 9.885895),
            Point(146.207165, 18.815694, 4.676763),
            Point(149.810987, 21.502249, 17.083618),
            Point(123.62156, 21.64297, 17.733487),
            Point(143.846461, 21.388459, 16.558122),
            Point(123.531701, 20.756807, 13.641072),
            Point(131.906468, 20.861163, 14.123002),
            Point(134.607416, 15.741609, -9.519754),
            Point(131.232551, 20.580328, 12.826068),
            Point(122.959207, 20.008058, 10.183251),
            Point(129.590699, 18.785461, 4.537144),
            Point(122.199489, 18.387709, 2.700273),
            Point(127.811443, 17.061665, -3.423569),
            Point(132.763864, 14.563498, -14.960425),
            Point(137.425771, 15.246311, -11.807104),
            Point(133.976431, 19.415144, 7.445098),
            Point(141.147246, 14.792123, -13.904602),
            Point(134.321155, 13.729688, -18.811062),
            Point(126.514179, 16.245401, -7.193181),
            Point(122.064196, 16.141604, -7.672527),
            Point(129.662786, 15.027072, -12.81958),
            Point(125.390575, 14.236865, -16.468853),
        };
        Mesh m = RemeshCDT::from_polylines({Polyline(border)});

        MINI_CHECK(m.is_valid());
    }


    MINI_TEST("RemeshCDT", "Irregular tilted polyline with holes.") {
        // uncomment #include "mesh.h"
        // uncomment #include "polyline.h"
        // uncomment #include "remesh_cdt.h"

        std::vector<Point> border = {
            Point(80.805571, 2.103432, 0),
            Point(77.056348, 6.382318, 0),
            Point(73.469325, 10.712368, 0),
            Point(76.913186, 20.429093, 0),
            Point(81.089809, 22.150636, 0),
            Point(80.489618, 16.428762, 0),
            Point(77.593256, 13.859214, 0),
            Point(79.137808, 10.314549, 0),
            Point(81.991647, 13.383871, 0),
            Point(82.6268, 17.180658, 0),
            Point(83.346466, 22.810648, 0),
            Point(80.646064, 27.128804, 0),
            Point(75.409533, 25.864862, 0),
            Point(72.096289, 20.679753, 0),
            Point(69.174993, 24.651843, 0),
            Point(73.459453, 30.365867, 0),
            Point(78.285356, 32.047254, 0),
            Point(84.294064, 31.313912, 0),
            Point(88.751527, 27.70307, 0),
            Point(89.863118, 24.247859, 0),
            Point(86.375694, 25.536811, 0),
            Point(85.671769, 28.704182, 0),
            Point(83.931992, 29.140561, 0),
            Point(83.87182, 26.993861, 0),
            Point(85.013894, 23.331116, 0),
            Point(86.696592, 22.050319, 0),
            Point(91.11007, 21.775838, 0),
            Point(93.068864, 24.22017, 0),
            Point(90.710749, 28.565741, 0),
            Point(91.638895, 29.907121, 0),
            Point(98.350352, 28.577586, 0),
            Point(97.932933, 19.2177, 0),
            Point(92.735476, 16.088655, 0),
            Point(88.933069, 18.173691, 0),
            Point(87.14089, 18.175371, 0),
            Point(88.303603, 12.293181, 0),
            Point(95.930447, 12.110783, 0),
            Point(99.851107, 14.725665, 0),
            Point(100.978975, 20.024267, 0),
            Point(102.243884, 24.855315, 0),
            Point(107.033349, 22.799502, 0),
            Point(106.609681, 11.898217, 0),
            Point(103.249112, 7.789628, 0),
            Point(96.095448, 7.274641, 0),
            Point(89.385773, 5.965419, 0),
            Point(96.860717, 2.216133, 0),
            Point(100.861767, 4.379475, 0),
            Point(101.881544, -0.42956, 0),
            Point(97.45084, -3.301412, 0),
            Point(91.973637, -1.202488, 0),
            Point(85.52513, 1.867323, 0),
            Point(84.223176, 6.692841, 0),
            Point(89.798972, 8.864768, 0),
            Point(95.492277, 9.295981, 0),
            Point(99.888391, 10.064787, 0),
            Point(103.261854, 11.705226, 0),
            Point(103.779131, 19.764949, 0),
            Point(102.711068, 14.994661, 0),
            Point(99.443654, 12.155559, 0),
            Point(97.732698, 11.322088, 0),
            Point(90.50953, 9.911877, 0),
            Point(85.903966, 9.907526, 0),
            Point(84.501578, 11.767742, 0),
            Point(85.498912, 16.396132, 0),
            Point(82.205875, 10.374609, 0),
            Point(79.29099, 8.223327, 0),
            Point(82.140367, 5.90496, 0),
            Point(80.805571, 2.103432, 0),
        };
        std::vector<Point> h1 = {
            Point(89.032991, 2.017152, 0),
            Point(87.733487, 4.228529, 0),
            Point(91.344022, 3.519583, 0),
            Point(94.484896, 1.422882, 0),
            Point(89.032991, 2.017152, 0),
        };
        std::vector<Point> h2 = {
            Point(92.03771, 19.272441, 0),
            Point(93.023809, 18.394685, 0),
            Point(95.237342, 19.201565, 0),
            Point(96.523355, 23.096688, 0),
            Point(96.499215, 27.739772, 0),
            Point(93.834079, 27.897701, 0),
            Point(94.368093, 26.068002, 0),
            Point(94.196483, 22.936949, 0),
            Point(92.03771, 19.272441, 0),
        };
        std::vector<Point> h3 = {
            Point(75.250389, 29.32022, 0),
            Point(71.879629, 25.409869, 0),
            Point(72.297014, 23.977747, 0),
            Point(74.564259, 26.656863, 0),
            Point(77.684277, 29.043429, 0),
            Point(81.250681, 29.040871, 0),
            Point(81.897777, 29.877526, 0),
            Point(80.430576, 30.631228, 0),
            Point(75.250389, 29.32022, 0),
        };
        std::vector<Point> h4 = {
            Point(78.759389, 19.25978, 0),
            Point(79.548907, 18.00544, 0),
            Point(77.949677, 16.331313, 0),
            Point(77.272732, 16.707375, 0),
            Point(78.759389, 19.25978, 0),
        };
        Mesh m = RemeshCDT::from_polylines({
            Polyline(border),
            Polyline(h1),
            Polyline(h2),
            Polyline(h3),
            Polyline(h4),
        }, false, false);

        MINI_CHECK(m.is_valid());
    }


} // namespace session_cpp

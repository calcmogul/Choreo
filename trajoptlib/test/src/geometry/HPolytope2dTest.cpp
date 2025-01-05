// Copyright (c) TrajoptLib contributors

#include <catch2/catch_test_macros.hpp>
#include <trajopt/geometry/HPolytope2.hpp>

TEST_CASE("HPolytope2d - Constructor", "[HPolytope2d]") {
  {
    trajopt::HPolytope2d P{std::vector<trajopt::Translation2d>{
        {1.0, -1.0}, {-1.0, -1.0}, {-1.0, 1.0}, {1.0, 1.0}}};

    CHECK(P.A().rows() == 4);
    CHECK(P.A().cols() == 2);
    CHECK(P.A() ==
          Eigen::Matrix<double, 4, 2>{{0, -2}, {-2, 0}, {0, 2}, {2, 0}});

    CHECK(P.b().rows() == 4);
    CHECK(P.b().cols() == 1);
    CHECK(P.b() == Eigen::Vector<double, 4>{{2}, {2}, {2}, {2}});

    CHECK(P.Contains({0.0, 0.0}));
    CHECK_FALSE(P.Contains({2.0, 0.0}));
    CHECK_FALSE(P.Contains({0.0, 2.0}));
    CHECK_FALSE(P.Contains({-2.0, 0.0}));
    CHECK_FALSE(P.Contains({0.0, -2.0}));
  }

  {
    trajopt::HPolytope2d P{std::vector<trajopt::Translation2d>{{0.0, -1.0},
                                                               {-1.0, 0.0},
                                                               {0.0, 1.0},

                                                               {1.0, 0.0}}};

    CHECK(P.A().rows() == 4);
    CHECK(P.A().cols() == 2);
    CHECK(P.A() ==
          Eigen::Matrix<double, 4, 2>{{-1, -1}, {-1, 1}, {1, 1}, {1, -1}});

    CHECK(P.b().rows() == 4);
    CHECK(P.b().cols() == 1);
    CHECK(P.b() == Eigen::Vector<double, 4>{{1}, {1}, {1}, {1}});

    CHECK(P.Contains({0.0, 0.0}));
    CHECK(P.Contains({1.0, 0.0}));
    CHECK_FALSE(P.Contains({1.0, 1.0}));
    CHECK(P.Contains({0.0, 1.0}));
    CHECK_FALSE(P.Contains({-1.0, 1.0}));
    CHECK(P.Contains({-1.0, 0.0}));
    CHECK_FALSE(P.Contains({-1.0, -1.0}));
    CHECK(P.Contains({0.0, -1.0}));
    CHECK_FALSE(P.Contains({1.0, -1.0}));
  }
}

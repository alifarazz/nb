#include "header/body_attrs.hpp"
#include <bits/stdc++.h>

class NBody {
  // TODO: fix data upload
public:
  constexpr static auto G = 6.67408e-11f;
  // const static inline auto G = 1.0f;

  // MASS, POSITION, VELOCITY, ACCELERATION
  BodyAttrs<float, float, float, float> ba;

  std::vector<float> forcesTable;

  void resolveCollisions() {
    auto resolveCollision = [&](std::size_t i, std::size_t j) {
      if (ba.px[i] == ba.px[j] && ba.py[i] == ba.py[j] &&
	  ba.pz[i] == ba.pz[j]) {
	std::swap(ba.vx[i], ba.vx[j]);
	std::swap(ba.vy[i], ba.vy[j]);
	std::swap(ba.vz[i], ba.vz[j]);
      }
    };

    for (std::size_t i = 0; i < ba.n; ++i)
      for (std::size_t j = i + 1; j < ba.n; ++j)
	resolveCollision(i, j);
  }

  void computeAccelerations() {
    auto computeAcceleration = [&](std::size_t i, std::size_t j) {
      // TODO: do something about sqrt
      const float x = ba.px[i] - ba.px[j];
      const float y = ba.py[i] - ba.py[j];

      const float len2 = x * x + y * y;
      const float intensity = G * ba.m[i] * ba.m[j] / len2;
      // const float leng = std::sqrt(len2);

      // std::tuple<float, float> normalized(x / leng, y / leng);

      // return std::make_tuple(std::get<0>(normalized) * intensity,
      //			   std::get<1>(normalized) * intensity);
      return intensity;
    };

    const auto n = ba.n;

    for (size_t i = 0; i < n; i++) {
      float *row = forcesTable.data() + n * i;

      for (size_t j = 0; j < n - i; j++) // REVIEW: stop condition
      {
	*(row + j) = (i == j) ? 0.0f : computeAcceleration(i, j);
      }
      for (size_t j = n - i; j < n; j++)
	*(row + j) = 0.0f;
    }
  }

  void computeVelocities() {
    auto computeVelocity = [&](int i) {
      ba.vx[i] += ba.ax[i];
      ba.vy[i] += ba.ay[i];
      ba.vz[i] += ba.az[i];
    };

    for (std::size_t i = 0; i < ba.n; i++) {
      computeVelocity(i);
    }
  }

  void computePositions() {
    auto computePosition = [&](int i) {
      ba.px[i] += ba.vx[i] + ba.ax[i] / 2;
      ba.py[i] += ba.vy[i] + ba.ay[i] / 2;
      ba.pz[i] += ba.vz[i] + ba.az[i] / 2;
    };

    for (std::size_t i = 0; i < ba.n; i++) {
      computePosition(i);
    }
  }

  void iterate() {
    resolveCollisions();
    computeAccelerations();
    computeVelocities();
    computePositions();
  }

  std::istream &upload(std::istream &in, std::size_t idx) {
    return in >> this->ba.m[idx] >> this->ba.px[idx] >> this->ba.py[idx] >>
	   this->ba.pz[idx];
  }

  NBody(int n) {
    ba.reserve(n);
    std::fill(ba.vx.data(), ba.vx.data() + n, 0);
    std::fill(ba.vy.data(), ba.vy.data() + n, 0);
    std::fill(ba.vz.data(), ba.vz.data() + n, 0);

    forcesTable.reserve(n * n);
  }
};

int main() {
  using namespace std;

  ios::sync_with_stdio(false);
  cin.tie(nullptr);

  int n, niters;

  cin >> n >> niters;
  NBody nbody(n);
  for (int i = 0; i < n; i++)
    nbody.upload(cin, static_cast<int>(i));

  for (int k = 0; k < niters; k++) {
    nbody.iterate();
    for (int i = 0; i < n; i++) {
      for (int j = 0; j < n; j++)
	cout << *(nbody.forcesTable.data() + n * i + j) << '\t';
      cout << '\n';
    }
  }

  return EXIT_SUCCESS;
}

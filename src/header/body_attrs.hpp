#include <vector>

template <typename MAS, typename POS, typename VELO, typename ACC>
struct BodyAttrs {
  std::size_t n;
  std::vector<MAS> m;
  std::vector<POS> px, py, pz;
  std::vector<VELO> vx, vy, vz;
  std::vector<ACC> ax, ay, az;


  void reserve(std::size_t N) {
    n = N;
    m.reserve(n);
    px.reserve(n); py.reserve(n); pz.reserve(n);
    vx.reserve(n); vy.reserve(n); vz.reserve(n);
    ax.reserve(n); ay.reserve(n); az.reserve(n);
  }
};

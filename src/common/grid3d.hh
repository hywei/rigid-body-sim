#ifndef GRID3D_HPP
#define GRID3D_HPP

template <typename T>
class Grid3D{
public:
  Grid3D() {m_nx = 0; m_ny = 0; m_nz = 0; m_data = new T[1];}
  Grid3D(const Grid3D &); // undefined to prevent copying
  ~Grid3D() {delete [] m_data;}

  T & operator()(int i, int j, int k) {
    return m_data[i + m_nx * j + (m_nx * m_ny) * k];
  }

  const T & operator()(int i, int j, int k) const {
    return m_data[i + m_nx * j + (m_nx * m_ny) * k];
  }

  void resize(int nx, int ny, int nz) {
    m_nx = nx; m_ny = ny; m_nz = nz;
    delete [] m_data;
    m_data = new T[m_nx * m_ny * m_nz];
  }

private:
  T * m_data;
  int m_nx, m_ny, m_nz;
};

#endif

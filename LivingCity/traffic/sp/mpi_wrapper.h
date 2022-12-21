#ifndef _ABM_MPI_H_
#define _ABM_MPI_H_

#ifdef USE_MPI
#include "mpi.h"
#endif

namespace abm {
template <typename Tdatatype, size_t Tnsize>
std::vector<std::array<Tdatatype, Tnsize>> gather_vector_arrays(
    std::vector<std::array<Tdatatype, Tnsize>> const& x) {
#ifdef USE_MPI
  MPI_Datatype arr_t;
  MPI_Type_vector(Tnsize, 1, 1, MPI_INT, &arr_t);
  MPI_Type_commit(&arr_t);

  int mpi_size;
  MPI_Comm_size(MPI_COMM_WORLD, &mpi_size);

  unsigned x_size = x.size();
  std::vector<int> x_sizes(mpi_size);
  MPI_Gather(&x_size, 1, MPI_INT, x_sizes.data(), 1, MPI_INT, 0,
             MPI_COMM_WORLD);

  std::vector<std::array<Tdatatype, Tnsize>> all_x(
      std::accumulate(x_sizes.begin(), x_sizes.end(), 0));
  auto x_scan = x_sizes;
  std::partial_sum(x_scan.begin(), x_scan.end(), x_scan.begin());
  x_scan.insert(x_scan.begin(), 0);

  MPI_Gatherv(x.data(), x.size(), arr_t, all_x.data(), x_sizes.data(),
              x_scan.data(), arr_t, 0, MPI_COMM_WORLD);

  MPI_Type_free(&arr_t);

  return all_x;
#else
  return x;
#endif
}

template <typename Tdatatype>
std::vector<Tdatatype> gather_vectors_ids(std::vector<Tdatatype> const& x) {
#ifdef USE_MPI

  int mpi_size;
  MPI_Comm_size(MPI_COMM_WORLD, &mpi_size);

  unsigned x_size = x.size();
  std::vector<int> x_sizes(mpi_size);
  MPI_Gather(&x_size, 1, MPI_INT, x_sizes.data(), 1, MPI_INT, 0,
             MPI_COMM_WORLD);

  std::vector<Tdatatype> all_x(
      std::accumulate(x_sizes.begin(), x_sizes.end(), 0));
  auto x_scan = x_sizes;
  std::partial_sum(x_scan.begin(), x_scan.end(), x_scan.begin());
  x_scan.insert(x_scan.begin(), 0);

  MPI_Gatherv(x.data(), x.size(), MPI_LONG_LONG_INT, all_x.data(),
              x_sizes.data(), x_scan.data(), MPI_LONG_LONG_INT, 0,
              MPI_COMM_WORLD);

  return all_x;
#else
  return x;
#endif
}

}  // namespace abm

#endif  // _ABM_MPI_H_

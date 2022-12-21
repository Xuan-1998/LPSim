#include <memory>

#ifdef USE_MPI
#include "mpi.h"
#endif

#include "graph.h"
#include "router.h"
using namespace std;

int main(int argc, char** argv) {

  int mpi_rank = 0;
  int mpi_size = 1;

#ifdef USE_MPI
  // Initialise MPI
  MPI_Init(&argc, &argv);
  MPI_Comm_rank(MPI_COMM_WORLD, &mpi_rank);
  MPI_Comm_size(MPI_COMM_WORLD, &mpi_size);
#endif

  const bool directed = true;
  auto graph = std::make_shared<abm::Graph>(directed);

  std::string od_file;
  if (argc == 3) {
    // Read MatrixMarket file
    const std::string filename = argv[1];
    od_file = argv[2];
    graph->read_graph_matrix_market(filename);
  } else {
#ifdef USE_MPI
    MPI_Abort(MPI_COMM_WORLD, 1);
#endif
  }

  // On MPI rank 0 create a router and fetch all OD pairs
  auto router = std::make_unique<abm::Router>(graph);
  router->read_od_pairs(od_file);

  const auto all_paths = router->compute_routes(mpi_rank, mpi_size);
  //printf("size = %lld\n", all_paths.size());
  ofstream fout;
  fout.open("index_path_vec.txt");
  for (int x = 0; x < all_paths.size(); x++){
      //printf("edge %d = %lld\n", x, ual[x]);
      fout << all_paths[x] << endl;
  }
  fout.close();
  if (mpi_rank == 0)
    std::cout << "Collected paths: " << all_paths.size() << std::endl;

#ifdef USE_MPI
  MPI_Finalize();
#endif
}

#!/bin/bash 
# Time
metrics="sm__cycles_elapsed.avg,\
sm__cycles_elapsed.avg.per_second,"

# DP
metrics+="sm__sass_thread_inst_executed_op_dadd_pred_on.sum,\
sm__sass_thread_inst_executed_op_dfma_pred_on.sum,\
sm__sass_thread_inst_executed_op_dmul_pred_on.sum,"

# SP
metrics+="sm__sass_thread_inst_executed_op_fadd_pred_on.sum,\
sm__sass_thread_inst_executed_op_ffma_pred_on.sum,\
sm__sass_thread_inst_executed_op_fmul_pred_on.sum,"

# HP
metrics+="sm__sass_thread_inst_executed_op_hadd_pred_on.sum,\
sm__sass_thread_inst_executed_op_hfma_pred_on.sum,\
sm__sass_thread_inst_executed_op_hmul_pred_on.sum,"

# Roofline
metrics+="sm__inst_executed.sum,\
smsp__thread_inst_executed.sum,\
smsp__inst_executed_op_global_ld.sum,\
smsp__inst_executed_op_global_st.sum,\
smsp__inst_executed_op_local_ld.sum,\
smsp__inst_executed_op_local_st.sum,\
smsp__inst_executed_op_shared_ld.sum,\
smsp__inst_executed_op_shared_st.sum,\
l1tex__t_sectors_pipe_lsu_mem_global_op_ld.sum,\
l1tex__t_sectors_pipe_lsu_mem_global_op_st.sum,\
l1tex__t_sectors_pipe_lsu_mem_local_op_ld.sum,\
l1tex__t_sectors_pipe_lsu_mem_local_op_st.sum,\
l1tex__data_pipe_lsu_wavefronts_mem_shared_op_ld.sum,\
l1tex__data_pipe_lsu_wavefronts_mem_shared_op_st.sum,\
lts__t_sectors_op_read.sum,\
lts__t_sectors_op_atom.sum,\
lts__t_sectors_op_red.sum,\
lts__t_sectors_op_write.sum,\
dram__sectors_read.sum,\
dram__sectors_write.sum,\
lts__t_sectors_aperture_sysmem_op_read.sum,\
lts__t_sectors_aperture_sysmem_op_write.sum,\
smsp__sass_inst_executed_op_global_ld.sum,\
smsp__sass_inst_executed_op_global_st.sum,"


# Tensor Core
metrics+="sm__inst_executed_pipe_tensor.sum,"

# DRAM, L2 and L1
metrics+="dram__bytes.sum,\
lts__t_bytes.sum,\
l1tex__t_bytes.sum"

ncu --metrics $metrics -k kernel_trafficSimulation --launch-count 50 --launch-skip 1000 --csv  ./LivingCity >output.csv 

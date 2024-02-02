import os
import numpy as np
import pandas as pd
from roofline import roofline

datadir = './Data'
files = [x for x in os.listdir(datadir) if x.endswith('.csv') and x.startswith('output')]
files.sort()
files = [os.path.join(datadir, file) for file in files]
dfs = {}
for file in files:
    tag, ext = os.path.splitext(os.path.basename(file))
    dfs[tag] = pd.DataFrame()
    with open(file, 'r') as f:
        cnt = 0
        while True:
            ln = f.readline()
            if not ln:
                break
            cnt += 1
            if 'Host Name' in ln:
                break
        df = pd.read_csv(file, skiprows=cnt - 1)

        # filter record for a kernel execution
        df = df[df['ID']== 1]

    # calculate "Time"
        new_rows = []
        for id_value in df['ID'].unique():
            df_id = df[df['ID'] == id_value]
            cycles = df_id[df_id['Metric Name'] == 'sm__cycles_elapsed.avg']['Metric Value'].values
            cycles_per_sec = df_id[df_id['Metric Name'] == 'sm__cycles_elapsed.avg.per_second']['Metric Value'].values
            # if find
            if len(cycles) > 0 and len(cycles_per_sec) > 0:
                time_value = cycles[0] / cycles_per_sec[0]
                new_row = df_id.iloc[0].copy()
                new_row['Metric Name'] = 'kernel_time'
                new_row['Metric Value'] = time_value
                new_rows.append(new_row)

        df_new_rows = pd.DataFrame(new_rows)
        df = pd.concat([df, df_new_rows], ignore_index=True)

        # df['kernel_time'] = df['sm__cycles_elapsed.avg'] / df['sm__cycles_elapsed.avg.per_second']
        dft = df.groupby(['Kernel Name', 'Metric Name']).sum()
        dfmetric = pd.pivot_table(dft, index='Kernel Name', columns='Metric Name', values='Metric Value')
        dfmetric['Count'] = df.groupby(['Kernel Name']).count()['ID'].div(dfmetric.shape[1])

        dfmetric['Time'] = dfmetric['kernel_time'] / dfmetric['Count']

        dfmetric['inst_executed'] =dfmetric['sm__inst_executed.sum']/ dfmetric['Count']
        dfmetric['inst_thread_executed'] = dfmetric['smsp__thread_inst_executed.sum']/ dfmetric['Count']
        dfmetric['inst_executed_global_loads'] = dfmetric['smsp__sass_inst_executed_op_global_ld.sum']/ dfmetric['Count']
        dfmetric['inst_executed_global_stores'] = dfmetric['smsp__sass_inst_executed_op_global_st.sum']/ dfmetric['Count']
        dfmetric['inst_executed_local_loads'] = dfmetric['smsp__inst_executed_op_local_ld.sum']/ dfmetric['Count']
        dfmetric['inst_executed_local_stores'] = dfmetric['smsp__inst_executed_op_local_st.sum']/ dfmetric['Count']
        dfmetric['inst_executed_shared_loads'] = dfmetric['smsp__inst_executed_op_shared_ld.sum']/ dfmetric['Count']
        dfmetric['inst_executed_shared_stores'] = dfmetric['smsp__inst_executed_op_shared_st.sum']/ dfmetric['Count']

        dfmetric['gld_transactions'] = dfmetric['l1tex__t_sectors_pipe_lsu_mem_global_op_ld.sum']/ dfmetric['Count']
        dfmetric['gst_transactions'] = dfmetric['l1tex__t_sectors_pipe_lsu_mem_global_op_st.sum']/ dfmetric['Count']
        dfmetric['local_load_transactions'] = dfmetric['l1tex__t_sectors_pipe_lsu_mem_local_op_ld.sum']/ dfmetric['Count']
        dfmetric['local_store_transactions'] = dfmetric['l1tex__t_sectors_pipe_lsu_mem_local_op_st.sum']/ dfmetric['Count']
        dfmetric['shared_load_transactions'] = dfmetric['l1tex__data_pipe_lsu_wavefronts_mem_shared_op_ld.sum']/ dfmetric['Count']
        dfmetric['shared_store_transactions'] = dfmetric['l1tex__data_pipe_lsu_wavefronts_mem_shared_op_st.sum']/ dfmetric['Count']

        dfmetric['l2_read_transactions'] = (dfmetric['lts__t_sectors_op_read.sum']+dfmetric['lts__t_sectors_op_atom.sum']+dfmetric['lts__t_sectors_op_red.sum'])/ dfmetric['Count']
        dfmetric['l2_write_transactions'] = (dfmetric['lts__t_sectors_op_write.sum'] + dfmetric['lts__t_sectors_op_atom.sum'] + dfmetric['lts__t_sectors_op_red.sum'])/ dfmetric['Count']
        dfmetric['dram_read_transactions'] = dfmetric['dram__sectors_read.sum']/ dfmetric['Count']
        dfmetric['dram_write_transactions'] = dfmetric['dram__sectors_write.sum']/ dfmetric['Count']
        dfmetric['sysmem_read_transactions'] = dfmetric['lts__t_sectors_aperture_sysmem_op_read.sum']/ dfmetric['Count']
        dfmetric['sysmem_write_transactions'] = dfmetric['lts__t_sectors_aperture_sysmem_op_write.sum']/ dfmetric['Count']




        dfmetric['warp_instruction_throughput']=dfmetric['inst_executed']/ dfmetric['Time']/ 1024 / 1024 / 1024
        dfmetric['loadstore_intensity']=(dfmetric['inst_executed_global_loads']+dfmetric['inst_executed_global_stores']).div(dfmetric['gld_transactions']+dfmetric['gst_transactions'])# /32??
        dfmetric['loadstore_performance'] = (dfmetric['inst_executed_global_loads']+dfmetric['inst_executed_global_stores'] ) / dfmetric['Time'] / 1024 / 1024 / 1024
        dfmetric['AI HBM'] = (dfmetric['inst_thread_executed']/32).div(dfmetric['dram_read_transactions']+dfmetric['dram_write_transactions'])
        dfmetric['AI L2'] = (dfmetric['inst_thread_executed']/32).div(dfmetric['l2_read_transactions']+dfmetric['l2_write_transactions'])
        dfmetric['AI L1'] = (dfmetric['inst_thread_executed']/32).div(dfmetric['gld_transactions']+dfmetric['gst_transactions']+dfmetric['shared_load_transactions']+dfmetric['shared_store_transactions'])
        dfmetric['Performance'] = (dfmetric['inst_thread_executed']/32)/ dfmetric['Time'] / 1024 / 1024 / 1024

        # dfmetric['CC FLOPs'] = 2 * dfmetric['sm__sass_thread_inst_executed_op_dfma_pred_on.sum'] \
        #                        + dfmetric['sm__sass_thread_inst_executed_op_dmul_pred_on.sum'] \
        #                        + dfmetric['sm__sass_thread_inst_executed_op_dadd_pred_on.sum'] \
        #                        + 2 * dfmetric['sm__sass_thread_inst_executed_op_ffma_pred_on.sum'] \
        #                        + dfmetric['sm__sass_thread_inst_executed_op_fmul_pred_on.sum'] \
        #                        + dfmetric['sm__sass_thread_inst_executed_op_fadd_pred_on.sum'] \
        #                        + 2 * dfmetric['sm__sass_thread_inst_executed_op_hfma_pred_on.sum'] \
        #                        + dfmetric['sm__sass_thread_inst_executed_op_hmul_pred_on.sum'] \
        #                        + dfmetric['sm__sass_thread_inst_executed_op_hadd_pred_on.sum']
        # dfmetric['TC FLOPs'] = 512 * dfmetric['sm__inst_executed_pipe_tensor.sum']
        # dfmetric['TC GFLOP/s'] = dfmetric['TC FLOPs'] / dfmetric['Time'] / 1024 / 1024 / 1024
        # dfmetric['all FLOPs'] = dfmetric['CC FLOPs'] + dfmetric['TC FLOPs']
        # dfmetric['AI HBM'] = dfmetric['all FLOPs'].div(dfmetric['dram__bytes.sum'])
        # dfmetric['AI L2'] = dfmetric['all FLOPs'].div(dfmetric['lts__t_bytes.sum'])
        # dfmetric['AI L1'] = dfmetric['all FLOPs'].div(dfmetric['l1tex__t_bytes.sum'])
        # dfmetric['Performance'] = dfmetric['all FLOPs'] / dfmetric['Time'] / 1024 / 1024 / 1024
        dfs[tag] = dfmetric

tags = dfs.keys()
flags = ['all']  # 'HBM','L2','L1' or 'all'
for tag in tags:
    for flag in flags:
        dfm = dfs[tag]
        LABELS = dfm.index.tolist()
        AIL1 = dfm['AI L1'].tolist()
        AIL2 = dfm['AI L2'].tolist()
        AIHBM = dfm['AI HBM'].tolist()
        Performance = dfm['Performance'].tolist()
        warp_instruction_throughput=dfm['warp_instruction_throughput'].tolist()
        loadstore_intensity = dfm['loadstore_intensity'].tolist()
        loadstore_performance = dfm['loadstore_performance'].tolist()


        roofline(tag, Performance, AIHBM, AIL2, AIL1, warp_instruction_throughput,loadstore_intensity,loadstore_performance,LABELS, flag)


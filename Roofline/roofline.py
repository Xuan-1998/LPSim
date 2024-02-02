import math

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

font = {'size': 15}
plt.rc('font', **font)

colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple', 'tab:brown', 'tab:pink', 'tab:gray',
          'tab:olive', 'tab:cyan']
styles = ['o', 's', 'v', '^', 'D', ">", "<", "*", "h", "H", "+", "1", "2", "3", "4", "8", "p", "d", "|", "_", ".", ","]

markersize = 10
markerwidth = 2
maxchar = 25


def roofline(filename, Peformance, AIHBM, AIL2=None, AIL1=None,warp_instruction_throughput=None,loadstore_intensity=None,loadstore_performance=None, LABELS=None, flag='HBM'):
    if not Peformance:
        print('Peformance can not be empty!')
        return
    if max(Peformance) == 0:
        print('Peformance are all 0s!')
        return
    if (not AIHBM) and (not AIL2) and (not AIL1):
        print('AIHBM, AIL2 and AIL1 can not all be empty!')
        return
    if (len(Peformance) != len(AIHBM)) or (len(Peformance) != len(AIL2)) or (len(Peformance) != len(AIL1)):
        print('Peformance needs to have the same length as AI!')
        return
    if (flag != 'HBM') and (flag != 'L2') and (flag != 'L1') and (flag != 'all'):
        print('flag needs to be one of HBM, L2, L1, and all!')
        return
    LABELS = [x[:maxchar] for x in LABELS]
    memRoofs = [('L1', 19287.3/128),('L2', 2212 / 32), ('HBM', 1190 / 32)]
    # memRoofs = [('L1', 19287.3/32), ('L2', 3877.6/32), ('HBM', 1346/32)]
    # cmpRoofs = [('Tensor', 96.9), ('DP', 7.8)]
    cmpRoofs = [ ('Theoretical Peak', 609.12)]
    fig = plt.figure(1, figsize=(10.67, 6.6))
    plt.clf()
    ax = fig.gca()
    ax.set_xscale('log')
    ax.set_yscale('log')
    ax.set_xlabel('Instuction Intensity (Warp Instructions per Transaction)')
    ax.set_ylabel('Performance (warp GIPS)')

    nx = 10000
    xmin = -3
    xmax = 3
    ymin = 0.01
    ymax = 2000

    ax.set_xlim(10 ** xmin, 10 ** xmax)
    ax.set_ylim(ymin, ymax)

    ixx = int(nx * 0.02)
    xlim = ax.get_xlim()
    ylim = ax.get_ylim()

    scomp_x_elbow = []
    scomp_ix_elbow = []
    smem_x_elbow = []
    smem_ix_elbow = []

    x = np.logspace(xmin, xmax, nx)

    # find elbow
    for roof in cmpRoofs:
        for ix in range(1, nx):
            if float(memRoofs[0][1] * x[ix]) >= roof[1]  and (memRoofs[0][1] * x[ix - 1]) < roof[1]:
                scomp_x_elbow.append(x[ix - 1])
                scomp_ix_elbow.append(ix - 1)
                break

    for roof in memRoofs:
        for ix in range(1, nx):
            if (cmpRoofs[0][1] <= roof[1] * x[ix] and cmpRoofs[0][1] > roof[1] * x[ix - 1]):
                smem_x_elbow.append(x[ix - 1])
                smem_ix_elbow.append(ix - 1)
                break
    # draw roofline
    for i in range(len(cmpRoofs)):
        roof = cmpRoofs[i][1]
        y = np.ones(len(x)) * roof
        ax.plot(x[scomp_ix_elbow[i]:], y[scomp_ix_elbow[i]:], c='k', ls='-', lw='2')

    for i in range(len(memRoofs)):
        roof = memRoofs[i][1]
        y = x * roof
        ax.plot(x[:smem_ix_elbow[i] + 1], y[:smem_ix_elbow[i] + 1], c='k', ls='-', lw='2')

    # # global memory wall
    # stride0_x = 1
    # stride0_y = 0
    # for roof in memRoofs:
    #     y_at_x = roof[1] * stride0_x
    #     if y_at_x > stride0_y:
    #         stride0_y = y_at_x
    # if stride0_y>cmpRoofs[0][1]:
    #     stride0_y=cmpRoofs[0][1]
    # ax.vlines(x=stride0_x, ymin=0, ymax=stride0_y, color='r', linestyle='-')
    # ax.text(stride0_x, 10**((math.log10(stride0_y)+math.log10(ymin))/2), 'stride-0', rotation='vertical', verticalalignment='center', color='r')

    #global memory wall - stride1
    stride1_x = 1
    stride1_y = 0
    for roof in memRoofs:
        y_at_x = roof[1] * stride1_x
        if y_at_x > stride1_y:
            stride1_y = y_at_x
    if stride1_y > cmpRoofs[0][1]:
        stride1_y = cmpRoofs[0][1]
    ax.vlines(x=stride1_x, ymin=0, ymax=stride1_y, color='r', linestyle='-')
    ax.text(stride1_x, 10 ** ((math.log10(stride1_y) + math.log10(ymin)) / 2), 'stride-1', rotation='vertical',
            verticalalignment='center', color='r')

    stride8_x = 1 / 32
    stride8_y = 0
    for roof in memRoofs:
        y_at_x = roof[1] * stride8_x
        if y_at_x > stride8_y:
            stride8_y = y_at_x
    if stride8_y > cmpRoofs[0][1]:
        stride8_y = cmpRoofs[0][1]
    ax.vlines(x=stride8_x, ymin=0, ymax=stride8_y, color='r', linestyle='-')
    ax.text(stride8_x, 10 ** ((math.log10(stride8_y) + math.log10(ymin)) / 2), 'stride-8', rotation='vertical',
            verticalalignment='center', color='r')

    # draw data points
    for i in range(len(AIHBM)):
        if flag == 'L1':
            ax.plot(float(AIL1[i]), float(Peformance[i]), c=colors[i % 10], marker=styles[0], \
                    linestyle='None', ms=markersize, markerfacecolor='none', \
                    markeredgewidth=markerwidth, label=LABELS[i] if LABELS else "unknown")
        elif flag == 'L2':
            ax.plot(float(AIL2[i]), float(Peformance[i]), c=colors[i % 10], marker=styles[1], \
                    linestyle='None', ms=markersize, markerfacecolor='none', \
                    markeredgewidth=markerwidth, label=LABELS[i] if LABELS else "unknown")
        elif flag == 'HBM':
            ax.plot(float(AIHBM[i]), float(Peformance[i]), c=colors[i % 10], marker=styles[2], \
                    linestyle='None', ms=markersize, markerfacecolor='none', \
                    markeredgewidth=markerwidth, label=LABELS[i] if LABELS else "unknown")
        elif flag == 'all':
            ax.plot(float(AIL1[i]), float(Peformance[i]), c=colors[i % 10], marker=styles[0], \
                    linestyle='None', ms=markersize, markerfacecolor='none', \
                    markeredgewidth=markerwidth, label=LABELS[i] if LABELS else "unknown")
            ax.plot(float(AIL2[i]), float(Peformance[i]), c=colors[i % 10], marker=styles[1], \
                    linestyle='None', ms=markersize, markerfacecolor='none', \
                    markeredgewidth=markerwidth, label=LABELS[i] if LABELS else "unknown")
            ax.plot(float(AIHBM[i]), float(Peformance[i]), c=colors[i % 10], marker=styles[2], \
                    linestyle='None', ms=markersize, markerfacecolor='none', \
                    markeredgewidth=markerwidth, label=LABELS[i] if LABELS else "unknown")

        # global memory wall point
        ax.plot(float(loadstore_intensity[i]), float(loadstore_performance[i]), c='orange', marker=styles[0], \
                    linestyle='None', ms=markersize, markerfacecolor='none', \
                    markeredgewidth=markerwidth, label=LABELS[i] if LABELS else "unknown")


    # thread predication line
    ax.axhline(y=warp_instruction_throughput, color='grey', linestyle='--')
    marker_handles = []

    # legend
    if flag == 'L1':
        marker_handles.append(ax.plot([], [], c='k', marker=styles[0], linestyle='None', ms=markersize, \
                                      markerfacecolor='none', markeredgewidth=markerwidth, label=memRoofs[0][0])[0])
    elif flag == 'L2':
        marker_handles.append(ax.plot([], [], c='k', marker=styles[1], linestyle='None', ms=markersize, \
                                      markerfacecolor='none', markeredgewidth=markerwidth, label=memRoofs[1][0])[0])
    elif flag == 'HBM':
        marker_handles.append(ax.plot([], [], c='k', marker=styles[2], linestyle='None', ms=markersize, \
                                      markerfacecolor='none', markeredgewidth=markerwidth, label=memRoofs[2][0])[0])
    elif flag == 'all':
        for i in range(3):
            marker_handles.append(ax.plot([], [], c='k', marker=styles[i], linestyle='None', ms=markersize, \
                                          markerfacecolor='none', markeredgewidth=markerwidth, label=memRoofs[i][0])[0])

    # text
    for roof in cmpRoofs:
        ax.text(x[-ixx], roof[1] ,
                roof[0] + ': ' + '{0:.1f}'.format(roof[1]) + ' warp GIPS',
                horizontalalignment='right',
                verticalalignment='bottom')

    for roof in memRoofs:
        ang = np.arctan(np.log10(xlim[1] / xlim[0]) / np.log10(ylim[1] / ylim[0])
                        * fig.get_size_inches()[1] / fig.get_size_inches()[0])
        if x[ixx] * roof[1] > ymin:
            ax.text(x[ixx], x[ixx] * roof[1] * (1 + 0.25 * np.sin(ang) ** 2),
                    roof[0] + ': ' + '{0:.1f}'.format(float(roof[1])) + ' GTXN/s',
                    horizontalalignment='left',
                    verticalalignment='bottom',
                    rotation=180 / np.pi * ang)
        else:
            ymin_ix_elbow = list()
            ymin_x_elbow = list()
            for ix in range(1, nx):
                if (ymin <= roof[1] * x[ix] and ymin > roof[1] * x[ix - 1]):
                    ymin_x_elbow.append(x[ix - 1])
                    ymin_ix_elbow.append(ix - 1)
                    break
            ax.text(x[ixx + ymin_ix_elbow[0]], x[ixx + ymin_ix_elbow[0]] * roof[1] * (1 + 0.25 * np.sin(ang) ** 2),
                    roof[0] + ': ' + '{0:.1f}'.format(float(roof[1])) + ' GTXN/s',
                    horizontalalignment='left',
                    verticalalignment='bottom',
                    rotation=180 / np.pi * ang)

    leg1 = plt.legend(handles=marker_handles, loc='lower right', ncol=len(flag[0]) if 'all' not in flag else 3,
                      bbox_to_anchor=(1, 0))
    ax.add_artist(leg1)

    # legend of kernel function name

    # patch_handles = list()
    # for i in range(0, len(AIHBM)):
    #     if Peformance[i] > 0:
    #         patch_handles.append(mpatches.Patch(color=colors[i % 10], label=LABELS[i] if LABELS else "unknown"))

    # leg2 = plt.legend(handles=patch_handles, loc=4, ncol=1, bbox_to_anchor=(1, 0.1), scatterpoints=1)

    # ax.text(xlim[0] * 1.1, ylim[1] / 1.1, '-'.join([filename, flag]), horizontalalignment='left',
    #         verticalalignment='top')
    #     plt.title('-'.join([filename,flag]))

    # plt.savefig('_'.join([filename, flag]) + '.png')
#     plt.savefig('_'.join([filename,flag])+'.eps')
    plt.show()



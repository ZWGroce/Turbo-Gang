# import matplotlib.pyplot as plt
# import numpy as np
# import tkinter as tk
import csv
from bisect import bisect_left
# from scipy.interpolate import make_interp_spline


def pcomp(mdotair, speed):
    x = []
    y = []
    z = []

    rot = speed/4241

    with open("C:/Users/Zachary Groce/Documents/My Documents/School/1c Year/Capstone/Mechanical Engineering Design"
              "/LabVIEW/Support/cmap.csv", 'r') as readFile:
        reader = csv.reader(readFile)
        lines = list(reader)
        linecount = 0
        for row in lines:
            if linecount == 0:
                del row[0]
                x = [float(i) for i in row]
                linecount += 1
            else:
                y.append(float(row[0]))
                del row[0]
                z.append([float(i) for i in row])
                linecount += 1

    pos_omega = bisect_left(x, rot)

    if pos_omega > 8:
        pos_omega = 8
    elif pos_omega < 1:
        pos_omega = 1
    pos_mair = bisect_left(y, mdotair)

    x1 = x[pos_omega - 1]
    x2 = x[pos_omega]
    y1 = y[pos_mair - 1]
    y2 = y[pos_mair]

    q11 = z[pos_mair - 1][pos_omega - 1]
    q12 = z[pos_mair - 1][pos_omega]
    q21 = z[pos_mair][pos_omega - 1]
    q22 = z[pos_mair][pos_omega]

    pratio = (1/((x2-x1)*(y2-y1)))*((q11*(x2-rot)*(y2-mdotair))+(q21*(rot-x1)*(y2-mdotair))+(q12*(x2-rot)*(mdotair-y1))
                                    + (q22*(rot-x1)*(mdotair-y1)))
    return pratio


def fc_initial(qgen):
    counter = 0
    mair = (0.5/1500)
    hht = 302
    aht = 0.06
    cpcell = 662
    tair = 1000
    tcell = 1000
    tdyn = 1
    tin = 1073
    tstep = 10
    time = [0]
    tairout = [tin]

    while counter < 600:
        tcell = ((tstep/cpcell)*(qgen - (hht*aht*(tcell - ((tair + tin)/2)))))+tcell
        cpair = (0.1883*tair) + 943.5
        tass = (((hht*aht)-(2*mair*cpair))/(2*mair*cpair*hht*aht))*qgen
        tdyn = (((hht*aht*tstep)/cpcell)*(tass-tdyn))+tdyn
        tair = tcell + tdyn

        time.append(counter)
        tairout.append(tair)
        counter += tstep

    temps = [tcell, tdyn, tair]
    return temps

'''
def plot_data(time, data1, data2, data3, data4):
    times = np.linspace(min(time), max(time), 300)
    # spl = make_interp_spline(times, array, k=5)
    # array_new = spl(times)

    plt.subplot(221)
    spl2 = make_interp_spline(time, data2, k=3)
    data2_new = spl2(times)
    plt.plot(times, data2_new, color='red')
    # plt.grid()
    plt.title('Shaft Speed')
    plt.ylabel('Shaft Speed [rpm]')
    plt.text(time[-1] - .5, data2[-1] - 1500, int(data2[-1]))

    plt.subplot(222)
    plt.plot(time, data4, color='darkorchid')
    # plt.grid()
    plt.title('Compressor Work')
    plt.ylabel('Compressor Work [KW]')
    plt.text(time[-1] - .3, data4[-1] - 25, int(data4[-1]))

    plt.subplot(223)
    spl1 = make_interp_spline(time, data1, k=3)
    data1_new = spl1(times)
    plt.plot(times, data1_new)
    # plt.grid()
    plt.title('Mass Flowrate of Air')
    plt.ylabel('Mass Flowrate [kg/s]')
    plt.xlabel('Time [s]')
    plt.text(time[-1] - .3, data1[-1] - 0.125, "{:.2f}".format(data1[-1]))

    plt.subplot(224)
    plt.plot(time, data3, color='green')
    # plt.grid()
    plt.title('Fuel Cell Outlet Air Temperature')
    plt.ylabel('Fuel Cell Outlet\nTemperature [K]')
    plt.xlabel('Time [s]')
    plt.text(time[-1] - .35, data3[-1] + 5, int(data3[-1]))
    plt.show()
'''


def gt_initial(qgen, tatm, speed):
    counter = 0
    tstep = 0.1
    tstepFC = tstep * 100
    mair = 0.5
    pratiocomp = 4
    turbEff = 0.7
    compEff = 0.65
    kair = 1.4
    wcomp = 5
    tcomp = tatm
    welec = 45
    ival = 0.027
    omega = 2094
    desired_speed = (-0.0000001085*(speed**2))+(0.1187365293*speed)-225.0940783
    last_speed = 0
    kp = 0.15
    kd = 0.01

    temps = fc_initial(150)
    tcell = temps[0]
    tdyn = temps[1]
    tair = temps[2]
    cpcell = 662
    hht = 302
    aht = 0.06

    '''mass_flow = [mair]
    shaft_speed = [(omega*9.550)]
    t_air_out = [tair]
    p_comp = [pratiocomp]
    w_comp = [wcomp]
    times = [0]'''

    while counter < 5:
        cpair = ((0.1883 * tair) + 943.5)/1000
        pratioturb = 0.9 * pratiocomp
        wturb = turbEff * mair * cpair * tair * (1-(1/pratioturb)**((kair - 1)/kair))
        wlosses = 0.1 * wturb
        wtot = (wturb - wcomp - wlosses - welec) * 1000
        current_speed = (((wtot/ival)*tstep)/omega) + omega
        error = desired_speed - current_speed
        up = kp * error
        ud = kd * ((current_speed - last_speed)/tstep)
        utot = up + ud
        omega = utot + omega
        last_speed = current_speed
        mair = 0.000416 * omega
        wcomp = (0.009267 * mair * omega**2)/1000
        pratiocomp = pcomp(mair, omega)
        tcomp = tatm + (wcomp / (compEff*cpair*mair))

        tcell = ((tstepFC/cpcell)*(qgen - (hht*aht*(tcell - ((tair + (tcomp + 700))/2)))))+tcell
        cpair = cpair * 1000
        tass = (((hht * aht) - (2 * (mair/1500) * cpair)) / (2 * (mair/1500) * cpair * hht * aht)) * qgen
        tdyn = (((hht * aht * tstepFC) / cpcell) * (tass - tdyn)) + tdyn
        tair = tcell + tdyn

        counter += tstep
        '''mass_flow.append(mair)
        shaft_speed.append((omega * 9.550))
        t_air_out.append(tair)
        p_comp.append((pratiocomp*100))
        w_comp.append(wcomp)
        times.append(counter)'''

    # plot_data(times, mass_flow, shaft_speed, t_air_out, w_comp)
    vals = [pratiocomp, tair, mair, omega, tcomp, wcomp, tcell, tdyn]
    return vals


def single_run(inputs):
    voltage = 0.7

    if inputs[0] == 0:
        qgen = inputs[1]
        tatm = inputs[2]
        speed = inputs[3]
        currentDraw = inputs[4]
        inputs.clear()
        inputs = gt_initial(qgen, tatm, speed)
        pratiocomp = inputs[0]
        tair = inputs[1]
        mair = inputs[2]
        omega = inputs[3]
        wcomp = inputs[5]
        tcell = inputs[6]
        tdyn = inputs[7]
    else:
        tatm = inputs[1]
        pratiocomp = (inputs[2]/100)
        tair = inputs[3]
        mair = inputs[4]
        omega = inputs[5]
        wcomp = inputs[6]
        tcell = inputs[7]
        tdyn = inputs[8]
        qgenLast = inputs[9]
        currentDraw = inputs[10]
        currentLast = inputs[11]

        if currentDraw != currentLast:
            qgen = qgenLast + (currentDraw - currentLast) * voltage
        else:
            qgen = qgenLast


    tstep = 0.1
    tstepFC = tstep * 100
    turbEff = 0.7
    compEff = 0.65
    kair = 1.4
    welec = 45
    ival = 0.027
    desired_speed = 4241
    kp = 0.25
    cpcell = 662
    hht = 302
    aht = 0.06

    cpair = ((0.1883 * tair) + 943.5) / 1000
    pratioturb = 0.9 * pratiocomp
    wturb = turbEff * mair * cpair * tair * (1 - (1 / pratioturb) ** ((kair - 1) / kair))
    wlosses = 0.1 * wturb
    wtot = (wturb - wcomp - wlosses - welec) * 1000
    current_speed = (((wtot / ival) * tstep) / omega) + omega
    error = desired_speed - current_speed
    up = kp * error
    omega = omega + up
    if omega > 4241:
        omega = 4241
    mair = 0.000416 * omega
    wcomp = (0.009267 * mair * omega ** 2) / 1000
    pratiocomp = pcomp(mair, omega)
    tcomp = tatm + (wcomp / (compEff * cpair * mair))

    tcell = ((tstepFC / cpcell) * (qgen - (hht * aht * (tcell - ((tair + (tcomp + 700)) / 2))))) + tcell
    cpair = cpair * 1000
    tass = (((hht * aht) - (2 * (mair / 1500) * cpair)) / (2 * (mair / 1500) * cpair * hht * aht)) * qgen
    tdyn = (((hht * aht * tstepFC) / cpcell) * (tass - tdyn)) + tdyn
    tair = tcell + tdyn

    qgenLast = qgen
    currentLast = currentDraw
    outputs = [(pratiocomp*100), tair, mair, omega, tcomp, wcomp, wturb, tcell, tdyn, qgenLast, currentLast]

    return outputs


'''firstinput = [0, 150, 288, 40500, 1500]
firstoutput = single_run(firstinput)
firstoutput.insert(0, 1)
firstoutput.insert(1, 288)
firstoutput.pop(6)
firstoutput.pop(7)
firstoutput.insert(10, 1500)
print("This is the first output")
for i in firstoutput:
    print(i)
secondoutput = single_run(firstoutput)
print("This is the second output")
for i in secondoutput:
    print(i)'''

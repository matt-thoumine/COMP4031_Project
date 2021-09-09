import tkinter as tk
import pandas as pd
import matplotlib.pyplot as plt
import scipy.stats as stats

import g
from rider import Rider
from path import Path

import cProfile
import pstats


def setup(window):
    window.resizable(False,False)
    canvas = tk.Canvas(window,width=1900,height=1000)
    canvas.pack()
    return canvas

def register(canvas,No_Of_Riders,wpk,coop):
    
    reg_Riders = []
    
    #Set Up Measured Rider for Hoenigman tests
    rider = Rider("Rider_m")
    rider.watts_per_kilo = wpk
    rider.cooperate = coop
    reg_Riders.append(rider)
    #Comment in line below for graphical representation
    #rider.display(canvas)
    
    #Define Distribution Of other Riders
    wpk_lower = 6.3
    wpk_upper = 8.3
    wpk_mu = 7.1
    wpk_sigma = 0.4 
    watts_per_kilo = stats.truncnorm.rvs((wpk_lower-wpk_mu)/wpk_sigma, (wpk_upper-wpk_mu)/wpk_sigma, loc=wpk_mu, scale=wpk_sigma, size=No_Of_Riders)
    
    coop_lower = 0
    coop_upper = 1
    coop_mu = 0.48
    coop_sigma = 0.2
    coop_tendencies = stats.truncnorm.rvs((coop_lower-coop_mu)/coop_sigma, (coop_upper-coop_mu)/coop_sigma, loc=coop_mu, scale=coop_sigma, size=No_Of_Riders)
    
    for i in range(0,No_Of_Riders-1):
        rider = Rider("Rider"+str(i))
        rider.watts_per_kilo = watts_per_kilo[i]
        rider.cooperate = coop_tendencies[i]
        reg_Riders.append(rider)
        #Comment in line below for graphical representation
        #rider.display(canvas)
        
    path = Path("Path")
    path.display(canvas)
    
    return reg_Riders, path

def group_checker(gp,groups,path):
    #Checks distances between rider normal points on route, if less than or equal to 5 then same group
    
    for r in gp:
        for group in groups:
            for rider in group:
                r_on_path = g.getNormalPoint(r.location,path.points[r.path_int],path.points[r.path_int+1])
                rider_on_path = g.getNormalPoint(rider.location,path.points[rider.path_int],path.points[rider.path_int+1])
                if g.distance(r_on_path,rider_on_path) <= 5:
                    groups.remove(group)
                    group = list(set(gp+group))
                    group_checker(group,groups,path)
                    return
    groups.append(gp)
                
def group(reg_Riders):
    
    #Place Riders into Groups based off distance between Riders
    groups = []
           
    for r in reg_Riders:
        groups.append([r,r.dist_remaining])
    
    groups.sort(key = lambda x: x[1])
    
    group_number = 1
    
    for i in range(0,len(groups)):
        if i != 0 and (groups[i][1] - groups[i-1][1]) > 3:
            group_number += 1
        groups[i][0].rider_group = group_number
        
    return groups
                      

def Iterate(canvas,reg_Riders,window,path,end_results,time_results,No_Of_Riders,t,run_number):
    t += 1
    avg_location = 0
    
    group(reg_Riders)
    
    
    for rider in reg_Riders:
        rider.applyBehaviour(path,reg_Riders,canvas)
        
        #Remove Finished Riders From Simulation
        if rider.dist_remaining == 0:
            #Record Results of Rider for Hoenigman test
            if rider.name == 'Rider_m':
                end_results.append([rider.watts_per_kilo,rider.cooperate,(No_Of_Riders+1)-len(reg_Riders),rider.avg_time_below_threshold])
            reg_Riders.remove(rider)
            continue
        
        rider.update(path,reg_Riders)
        avg_location = avg_location + rider.location
    
    #Stop Simulation if no riders are left
    if len(reg_Riders) == 0:
        window.destroy()
        return end_results, time_results
    
    # Calculate Average Rider locatio with respect to the start point - screen center
    # Allows for the scrolling screen when displaying graphics so riders don't just go out of view
    avg_location = avg_location/len(reg_Riders)
    diff = avg_location - g.course_start
    path.points = path.points - diff
    #Comment in line below for graphical representation
    #path.display(canvas)
    
    for rider in reg_Riders:
        rider.location = rider.location - diff
        # Saving Rider Data for Ratamero Tests
        if rider.power(reg_Riders) <= 0.5*rider.max_10_power:
            rider.avg_time_below_threshold += 1
        if rider.energy <= 100000:
            time_results.append({'run':run_number,'t':t,'rider':rider.name,'r_d':rider.rider_df,'energy':rider.energy,'status':2})
        if rider.attack > 1 or rider.turn > 1:
            time_results.append({'run':run_number,'t':t, 'rider':rider.name,'r_d':rider.rider_df,'energy':rider.energy,'status':1})   
        else:
            time_results.append({'run':run_number,'t':t,'rider':rider.name,'r_d':rider.rider_df,'energy':rider.energy,'status':0})
        #Comment in line below for graphical representation
        #rider.display(canvas)
        
    canvas.after(1,Iterate,canvas,reg_Riders,window,path,end_results,time_results,No_Of_Riders,t,run_number)
        
def experiment(No_Of_Riders,run_number,wpk,coop):
    window = tk.Tk()
    canvas = setup(window)
    reg_Riders, path = register(canvas,No_Of_Riders,wpk,coop)
    
    end_results = []
    time_results = []
    t=0
    Iterate(canvas,reg_Riders,window,path,end_results,time_results,No_Of_Riders,t,run_number)
    window.mainloop()
    
    return end_results, time_results
    
def manipulate_end(end_results,numberOfRiders,wpk,coop):
    #Post Processing of collect Hoenigman data
    df = pd.DataFrame()
    
    i=1
    for run in end_results:
        for rider in run:
            da = pd.DataFrame({'run': [i] , 'watts_per_kilo' : rider[0], 'co_operation' : rider[1], 'rider':rider[2], 'x':rider[3]})
            da.set_index(['run','rider'], inplace = True, drop=True)
            df = pd.concat([df,da])
        i +=1
    
    df.to_excel(f'end_{numberOfRiders}r_{wpk}wpk_{coop}coop.xlsx')

def manipulate_time(time_results,numberOfRiders,wpk,coop):
    #Post Processing of collected Ratamero data
    df = pd.DataFrame()

    print('Time results pending')

    df = pd.DataFrame()
    for run in time_results:
        for tick in run:
            da = pd.DataFrame(tick, index = ['run'])
            df = pd.concat([df,da])
    
    df.set_index(['run','t','rider'], inplace = True, drop=True)
    
    grouped = df.groupby(['t','status'])
    
    print('Attempting Grouping')

    Active = [[],[]]
    Non = [[],[]]
    Exhaust = [[],[]]
    for key, group in grouped:
        if key[1] == 0:
            Non[0].append(key[0])
            Non[1].append(group['r_d'].mean())
        if key[1] == 1:
            Active[0].append(key[0])
            Active[1].append(group['r_d'].mean())
        if key[1] == 2:
            Exhaust[0].append(key[0])
            Exhaust[1].append(len(group))

            
    print('Graphing')
    
    #Can export to excel file rather than straight to graph but write time is long due to quantity of data
    
    fig = plt.figure()
    fig.suptitle('Distribution of Drafting Coefficient for Active vs Non-Active Riders')
    ax = fig.add_subplot(111)
    ax.plot(Non[0],Non[1],label='Non')
    ax.plot(Active[0],Active[1],label='Active')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Average Drafting Coefficient')
    plt.legend()
    plt.show()
    
    fig = plt.figure()
    fig.suptitle('Number of Exhausted Riders')
    ax = fig.add_subplot(111)
    ax.plot(Exhaust[0],Exhaust[1])
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Number of Riders')
    plt.show()
    
        
def runSetOfExperiments(numberOfRuns, numberOfRiders,wpk,coop):
    set_end_results = []
    set_time_results = []
    for run_number in range(1,numberOfRuns+1):
        end, time = experiment(numberOfRiders,run_number,wpk,coop)
        print(f'Completed Run {run_number}/{numberOfRuns} for {numberOfRiders} riders')
        set_end_results.append(end)
        set_time_results.append(time)
    
    #Comment Out Either Line Below Depending on Desired Results:
    #Hoenigman:
    #manipulate_end(set_end_results,numberOfRiders,wpk,coop)
    #Ratamero:
    manipulate_time(set_time_results,numberOfRiders,wpk,coop)

    return

def runExperimentwithDifferentParameters():
    #Comment Out Either Set Below Depending on Desired Results:
    #Hoenigman:
    #wpk= [[6.3],[6.7],[7.1],[7.5],[7.9],[8.3]]
    #coop = [[0.2],[0.4],[0.6],[0.8]]
    #Ratamero:
    wpk = [[7.1]]
    coop = [[0.48]]
    
    for numberOfRiders in range(25,26,2): #Set to Run 1 round at 25 riders
        for i in wpk:
            for j in coop:
                runSetOfExperiments(1,numberOfRiders,i[0],j[0])
                print(f'Completed Set For: {numberOfRiders}x Riders, {i[0]} wpk, {j[0]} coop')


#Straight Forward Run
runExperimentwithDifferentParameters()

#Run with Profiling for Optimization if Desired:
'''
profiler = cProfile.Profile()
profiler.enable()
runExperimentwithDifferentParameters()
profiler.disable()
stats = pstats.Stats(profiler).sort_stats('ncalls')
stats.print_stats()
'''


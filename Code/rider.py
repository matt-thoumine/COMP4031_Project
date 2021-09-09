import numpy as np
import math, random
from g import mag, normalize, limit, angleBetween, distance, getNormalPoint, course_start, rho, g


class Rider():
    
    def __init__(self,namep):
        self.name = namep
        # Given as box to avoid mass seperation behaviour at start that takes time to correct from
        # Box position and velocity may need adapting to fit different routes
        self.location = np.array([random.randint(course_start[0]-50,course_start[0]+5),random.randint(course_start[1]-5,course_start[1]+5),course_start[2]])
        self.velocity = np.array([random.randint(0,5),random.randint(-1,1),0])
        self.acceleration = np.array([0,0,0])
        self.theta = np.arctan2(self.velocity[0],self.velocity[1])
        
        #Rider Properties
        self.mass = 70 #kg
        self.bike_mass = 6.8 #kg
        self.unit_mass = self.mass + self.bike_mass
        self.area = 0.423 #m^2 (Front Area Blocken)
        self.drag_coefficient = 0.692 
        self.rr_coefficient = 0.0053 
        self.watts_per_kilo = 6.3
        self.max_10_power = self.watts_per_kilo*self.mass
        self.recovery_power = self.max_10_power*0.5
        self.size = 1.5 #In meters, worth increasing if looking at graphically as this is very small
        self.maxspeed = 27.8 
        self.maxforce = 300
        self.energy = 500000 #Joules, amount of energy rider starts with worth adjusting with course
        
        #Current position on route
        self.path_int = 0
        
        #Fill colour of rider, useful for graphical representation for monitoring rider state
        self.f = 'blue'
        
        #Rider Current goal
        self.driving = 0
        self.attack = 0
        self.turn = 0
        
        
        self.rider_group = 0
        self.cooperate = 0
        
        self.dist_remaining = 1
        self.avg_time_to_exhaustion = 0
        self.avg_time_below_threshold = 0
        self.rider_df = 1
        
    def display(self,canvas):
        canvas.delete(self.name)
        
        points=[ 
                     
                 (self.location[0]+(self.size*np.sin(self.theta))), \
                 (self.location[1]+(self.size*np.cos(self.theta))), \
       
                 (self.location[0]+self.size*np.sin(self.theta+((2*math.pi)/3))), \
                 (self.location[1]+self.size*np.cos(self.theta+((2*math.pi)/3))), \
                     
                 (self.location[0]), \
                 (self.location[1]), \
                     
                 (self.location[0]+self.size*np.sin(self.theta-((2*math.pi)/3))), \
                 (self.location[1]+self.size*np.cos(self.theta-((2*math.pi)/3))), \
                ]
               
    
        canvas.create_polygon(points,fill=self.f,tags=self.name)
        #Can add below for energy level text but difficult to see for many agents
        #canvas.create_text(self.location[0],self.location[1],text=str(self.energyLvl), tags=self.name)
       
    def update(self,p,Riders):
    
        #Subtract power from energy level    
        power = self.power(Riders)
        self.energy -= power

        #Limit Rider Velocity first based on physiology and then max possible speed on bike (incase downhill)        
        self.exhaust(p,Riders)
        self.velocity = limit((self.velocity+self.acceleration),self.maxspeed)
        
        # Define route in order to get Z coordinate
        seg_start = p.points[self.path_int]
        if self.path_int >= len(p.points)-1:
            seg_finish = p.points[self.path_int]
        else:
            seg_finish = p.points[self.path_int+1]
        predict = self.location+self.velocity
        normalPoint = getNormalPoint(predict,seg_start,seg_finish)
        
        # Update location while inforcing z axes
        self.location = self.location + self.velocity
        self.location[2] = normalPoint[2]
        
        # Reset acceleration and update rider theta for graphical representation
        self.acceleration *= 0
        self.theta = np.arctan2(self.velocity[0],self.velocity[1])
    
    def exhaust(self,p,Riders):
        
        #Time Remaining of race and current power output
        time_remaining = self.dist_remaining / mag(self.velocity + self.acceleration)
        power = self.power_x((self.velocity+self.acceleration),self.drafting(Riders))
        
        # Under Finish conditions (similar to Hoenigman)
        # Increase velocity by increments until correct power output is reached - it is acknowledged that this method could do with improving
        if self.dist_remaining < 5000:
            if self.energy >= self.max_10_power*60:
                while power < self.max_10_power:
                    self.acceleration = self.acceleration + (0.1*normalize(self.velocity))
                    power = self.power_x((self.velocity+self.acceleration),self.drafting(Riders))
            elif self.energy >= self.max_10_power*30:
                 while power < 0.9*self.max_10_power:
                    self.acceleration = self.acceleration + (0.1*normalize(self.velocity))
                    power = self.power_x((self.velocity+self.acceleration),self.drafting(Riders))
            elif self.energy >= 0:
                while power < 0.7*self.max_10_power:
                    self.acceleration = self.acceleration + (0.1*normalize(self.velocity))
                    power = self.power_x((self.velocity+self.acceleration),self.drafting(Riders))
            else:
                while power < 0.5*self.max_10_power:
                    self.acceleration = self.acceleration + (0.1*normalize(self.velocity))
                    power = self.power_x((self.velocity+self.acceleration),self.drafting(Riders))
        
        # Rest of the race
        # If rider is exhausted limit power output
        elif self.energy <= 100000:
            if power >  0.5*self.max_10_power:
                time_exhaustion = np.exp(-6.351 * np.log(power/self.max_10_power)+2.478)
                
                if time_exhaustion < time_remaining:
                    while time_exhaustion < time_remaining:
                        self.acceleration = self.acceleration + (-0.1*normalize(self.velocity))
                        time_remaining = self.dist_remaining / mag(self.velocity+self.acceleration)
                        power = self.power_x((self.velocity+self.acceleration),self.drafting(Riders))
                        if power > 0.5*self.max_10_power:
                            break
                        time_exhaustion = np.exp(-6.351 * np.log(power/self.max_10_power)+2.478)
                
                else:
                    while time_exhaustion > time_remaining :
                        self.acceleration = self.acceleration + (0.1*normalize(self.velocity))
                        time_remaining = self.dist_remaining / mag(self.velocity+self.acceleration)
                        power = self.power_x((self.velocity+self.acceleration),self.drafting(Riders))
                        time_exhaustion = np.exp(-6.351 * np.log(power/self.max_10_power)+2.478)
        
           
        else:
            # Define power output by current rider goal
            if self.attack > 0:
                while power < 0.9*self.max_10_power:
                    self.acceleration = self.acceleration + (0.1*normalize(self.velocity))
                    power = self.power_x((self.velocity+self.acceleration),self.drafting(Riders))
                    
            elif self.turn > 0:
                i = 0
                while power < 0.8*self.max_10_power and i < 10000:
                    self.acceleration = self.acceleration + (0.1*normalize(self.velocity))
                    power = self.power_x((self.velocity+self.acceleration),self.drafting(Riders))
                    i += 1
                    
            elif power > 0.5*self.max_10_power:
                time_exhaustion = np.exp(-6.351 * np.log(power/self.max_10_power)+2.478)
                
                if time_exhaustion < time_remaining:
                    while time_exhaustion < time_remaining:
                        self.acceleration = self.acceleration + (-0.1*normalize(self.velocity))
                        time_remaining = self.dist_remaining / mag(self.velocity+self.acceleration)
                        power = self.power_x((self.velocity+self.acceleration),self.drafting(Riders))
                        if power > 0.5*self.max_10_power:
                            break
                        time_exhaustion = np.exp(-6.351 * np.log(power/self.max_10_power)+2.478)
                
                else:
                    while time_exhaustion > time_remaining :
                        self.acceleration = self.acceleration + (0.1*normalize(self.velocity))
                        time_remaining = self.dist_remaining / mag(self.velocity+self.acceleration)
                        power = self.power_x((self.velocity+self.acceleration),self.drafting(Riders))
                        time_exhaustion = np.exp(-6.351 * np.log(power/self.max_10_power)+2.478)
        
        
        # Update avg_time_to_exhaustion for result collection
        power = self.power_x((self.velocity+self.acceleration),self.drafting(Riders))
        if power > 0:
            self.avg_time_to_exhaustion += np.exp(-6.351 * np.log(power/self.max_10_power)+2.478)

             
    def seek(self, target):
        # Takes a defined target location and steers agent towards target
        desired = target - self.location
        desired = normalize(desired)
        # Multiply by max speed for fastest possible reaction
        desired = desired*self.maxspeed
        
        steer = desired - self.velocity
        return steer
    
    def seek_draft(self, riders, p):
        #Check for draft forward, backward, left, and right or rider's current position
        # Using velocity vector to get direction of travel
        modifier = 0.1 * normalize(self.velocity)
        rot_modifier = np.array([-1*modifier[1],modifier[0],modifier[2]])
        
        forward = self.s_drafting(modifier, riders)
        backward = self.s_drafting(-modifier, riders)
        left = self.s_drafting(rot_modifier, riders)
        right = self.s_drafting(-rot_modifier, riders)
        current = self.drafting(riders)
        
        # Checks best and moves agent in that direction
        best = min(forward,backward,left,right,current)
        if best >= 1:
            return self.seek(p.points[self.path_int+1])
        if forward == best:
            return self.seek(self.location+modifier)
        elif backward == best:
            return self.seek(self.location-modifier)
        elif left == left:
            return self.seek(self.location+rot_modifier)
        elif right == best:
            return self.seek(self.location-rot_modifier)
        else:
            return 0
        
    def s_drafting(self, modifier, riders):
        
        # Checks for draft of rider with modifier applied
        
        seek_dist = 20
        seek_angle = 90
        
        possible = []
        location = self.location + modifier
        
        for r in riders:
            if self != r:
                dist = distance(location, r.location)
                if dist <= seek_dist:
                    angle_vector = r.location - location
                    angle = np.degrees(angleBetween(self.velocity,angle_vector))
                    if angle <= seek_angle:
                        possible.append([r,dist,angle])
        
        for p in possible:
            for d in possible:
                if self != r:
                    if p[2] - d[2] < 0.1 or d[2]-p[2]<0.1:
                        if p[1] < d[1]:
                            possible.remove(d)
                        else:
                            possible.remove(p)
                            break
                        
        if len(possible) != 0:
            sum_draft_factor = 0
            for r in possible:
                sum_draft_factor += self.draft_factor(r)
            return 1-(len(possible) - sum_draft_factor)
        
        return 1
                        
    def drafting(self,riders):
        
        # Gives rider a draft based on location of riders around
        
        if len(riders) == 1:
            return 1
        
        draft_dist = 20
        draft_angle = 90
        
        drafts = []
        for r in riders:
            if self != r:
                dist = distance(self.location,r.location)
                if dist <= draft_dist:
                    angle_vector = r.location - self.location
                    angle= np.degrees(angleBetween(self.velocity,angle_vector))
                    if angle <= draft_angle:
                        drafts.append([r,dist,angle])
        
        for r in drafts:
            for d in drafts:
                if r != d:
                    if r[2] - d[2] < 0.1 or d[2] - r[2] < 0.1:
                        if r[1] < d[1]:
                            drafts.remove(d)
                        else:
                            drafts.remove(r)
                            break

        if len(drafts) != 0:
            sum_draft_factor = 0
            for r in drafts:
                sum_draft_factor += self.draft_factor(r)
            CF_draft = 1-(len(drafts) - sum_draft_factor)
            if CF_draft < 0:
                return 0
            else:
                return CF_draft
            
        return 1

    
    def draft_factor(self,r):
        
        if r[1] <= 20 or r[2] <= 90:
            dw = r[1] - self.size*2
            if dw > 0:
                angle_correction = 0.01*r[2]
                #Blocken:
                CF_draft = ((0.1738*(dw**3))-(1.6583*(dw**2))+(5.9617*dw)+60.721)*0.01 + angle_correction
                #Olds:
                #CF_draft = 0.62 - (0.0104*dw) - (0.00452*(dw**2)) + angle_correction
                if CF_draft < 1:
                    return CF_draft          
        return 1
        
    def follow(self,p,canvas):
        
        predict_distance = 15
        #Normalize to unit vector, and look ahead
        predict = normalize(self.velocity)*predict_distance
        predict_location = self.location + predict
        
        if self.path_int < len(p.points):
            seg_start, seg_finish = self.increment_path(p)

        normalPoint = getNormalPoint(predict_location,seg_start,seg_finish)

        if (normalPoint[0] < min(seg_start[0],seg_finish[0]) or normalPoint[0] > max(seg_start[0],seg_finish[0])) or (normalPoint[1] < min(seg_start[1],seg_finish[1]) or normalPoint[1] > max(seg_start[1],seg_finish[1])):
            normalPoint = seg_finish


        if distance(predict_location,normalPoint) > p.radius/2:
            route = seg_finish - seg_start
            route = normalize(route) * predict_distance
            target = normalPoint + route
            return self.seek(target)
        
        return 0
        
    
    def increment_path(self,p):
        # As more points added within route this becomes important for predicting changes in route
        
        if self.path_int >= (len(p.points)-1):
            return p.points[self.path_int], p.points[self.path_int]
        
        if distance(self.location, p.points[self.path_int+1]) < 40: #Value on the end controls how "far down the road" an agent can "look"
            self.path_int +=  1
            return self.increment_path(p)
        
        return p.points[self.path_int], p.points[self.path_int+1]
            
    def applyForce(self,force):
        #F = ma
        a = force/self.unit_mass 
        self.acceleration = self.acceleration + a

        
    def separation(self,Riders):
        # Seperation radius matching with published works
        desired_sep = 2
        
        v_sum = np.array([0,0,0])
        numClose = 0
        steer = np.array([0,0,0])
        
        for r in Riders:
            dist = distance(self.location,r.location)
            if dist > 0 and dist < desired_sep:
                diff = self.location - r.location
                diff = normalize(diff)
                diff = diff/dist
                v_sum = v_sum+diff
                numClose += 1
        
        if numClose > 0:
            v_sum = v_sum/numClose
            v_sum = normalize(v_sum)*self.maxspeed
            steer = v_sum - self.velocity
            
        return steer
    
    def coherence(self,Riders):
        # Coherence radius matching with published works
        desired_coh = 20
        
        v_sum = np.array([0,0,0])
        numFar = 0
        steer = np.array([0,0,0])
        
        for r in Riders:
            dist = distance(self.location,r.location)
            if dist > 0 and dist < desired_coh:
                v_sum = v_sum + r.location
                numFar += 1
        
        if numFar > 0:
            v_sum = v_sum/numFar
            v_sum = v_sum - self.location
            v_sum = normalize(v_sum)*self.maxspeed
            steer = v_sum - self.velocity
        
        return steer
    
    def resistive_forces(self,riders):
        if mag(self.velocity) == 0:
            gradient = 0
        else:
            gradient = self.velocity[2]/(math.sqrt(self.velocity[0]**2 + self.velocity[1]**2))
        

        F_drag = 0.5*rho*self.drag_coefficient*self.area*(np.linalg.norm(self.velocity)**2)*self.drafting(riders)
        F_rr = np.cos(np.arctan(gradient))*self.rr_coefficient*self.unit_mass*g
        F_wb = 0
        F_weight = self.unit_mass*g*np.sin(np.arctan(gradient))
        F_ke = 0
        return normalize(self.velocity)*-1*(F_drag+F_rr+F_wb+F_weight+F_ke)
    
    def power(self,riders):
        if mag(self.velocity) == 0:
            gradient = 0
        else:
            gradient = self.velocity[2]/(math.sqrt(self.velocity[0]**2 + self.velocity[1]**2))

        P_drag =  0.5*rho*self.drag_coefficient*self.area*(np.linalg.norm(self.velocity)**2)*self.drafting(riders)*np.linalg.norm(self.velocity)
        P_rr = np.cos(np.arctan(gradient))*self.rr_coefficient*self.unit_mass*g*np.linalg.norm(self.velocity)
        P_wb = 0
        P_pe = np.linalg.norm(self.velocity)*self.unit_mass*g*np.sin(np.arctan(gradient))
        P_ke = (0.5*self.unit_mass*((np.linalg.norm(self.velocity+self.acceleration))**2 - (np.linalg.norm(self.velocity))**2))/1
        
        self.rider_df = self.drafting(riders)
        
        return (P_drag + P_rr + P_wb + P_pe + P_ke)/1
    
    def power_x(self,velocity,draft):

        if mag(velocity) == 0:
            gradient = 0
        else:
            gradient = velocity[2]/(math.sqrt(velocity[0]**2 + velocity[1]**2))
        
        F_drag =  0.5*rho*self.drag_coefficient*self.area*(mag(velocity)**2)*draft
        P_drag = F_drag*mag(velocity)
        
        F_rr = np.cos(np.arctan(gradient))*self.rr_coefficient*self.unit_mass*g
        P_rr = F_rr*mag(velocity)
        
        P_wb = 0
        
        F_weight = self.unit_mass*g*np.sin(np.arctan(gradient))
        P_pe = mag(velocity)*F_weight
        
        P_ke = (0.5*self.unit_mass*((mag(velocity+self.acceleration))**2 - (mag(velocity))**2))/1
        
        E = 1
        
        return (P_drag + P_rr + P_wb + P_pe + P_ke)/E
    
    def goal(self,p,Riders):
        
        if self.path_int >= len(p.points)-1:
            self.dist_remaining = 0
            return
        else:
            self.dist_remaining = distance(self.location,p.points[self.path_int+1])
            for i in range(self.path_int+1,len(p.points)-1):
                self.dist_remaining += distance(p.points[i],p.points[i+1])
        
        # Decision of whether to co-operate or defect based on self.cooperate level
        option = ['co-operate','defect']
        choice = np.random.choice(option, 1, p=[self.cooperate,1-self.cooperate])
        
        
        # Choice over ridden if mid attack
        if self.attack > 0:
            if self.attack > 180:
                choice = 'defect'
            else:
                choice = 'co-operate'
                self.attack = 0
        
        if self.dist_remaining < 5000:
            self.driving = self.seek(p.points[self.path_int+1])
            return

        # Forces exhausted riders to seek a draft
        if self.energy <= 100000:
            self.turn = 0
            self.attack = 0
            self.driving = self.seek_draft(Riders, p)
        
        if choice == 'defect':
            if self.power_x(self.velocity,1) > 0.8*self.max_10_power:
                self.attack += 1
            else:
                #Active -> Attack
                force = self.seek_draft(Riders, p)
            
            if self.attack > 0:
                #Non-active
                force = self.seek(p.points[self.path_int+1])

        
        if choice == 'co-operate':

            self.attack = 0
            if self.turn < 300:
                #Active -> Pull turn
                force = self.pull_turn(p, Riders)
            else:
                #Non-active
                force = self.seek_draft(Riders, p)
            
        #Update driving force will force relating to the goal decided above.
        self.driving = force
            
        
    def pull_turn(self,p,Riders):
        seek_dist = 5
        seek_angle = 85
        #fop -> front of pack
        rider_fop = None
        fop_distance = 0
        #seek rider at front of group
        for r in Riders:
            if self !=r:
                angle_vector = r.location - self.location
                angle = np.degrees(angleBetween(self.velocity,angle_vector))
                if angle <= seek_angle:
                    dist = distance(self.location,r.location)
                    if dist <= seek_dist:
                        if dist > fop_distance:
                            fop_distance = dist
                            rider_fop = r
                                
        if rider_fop == None:
            self.turn += 1
            return self.seek(p.points[self.path_int+1])
            
        else:
            return self.seek(rider_fop.location)
        
        
    
    def applyBehaviour(self,path,Riders,canvas):
       
        self.goal(path,Riders)
        if self.dist_remaining == 0:
            return
        
        pathing_forces = self.follow(path,canvas)
        driving_forces = self.driving
        flocking_forces = self.separation(Riders)+self.coherence(Riders)
        resistive_forces = self.resistive_forces(Riders)
        total_forces = driving_forces+flocking_forces+pathing_forces+resistive_forces

        self.applyForce(total_forces)

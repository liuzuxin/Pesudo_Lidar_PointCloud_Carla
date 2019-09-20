"""
Script to send custom control commands to the vehicle, record and plot the response.
[For System Identification]

Author: Ashish Roongta
SafeAI lab
Carnegie Mellon University 
Copyright @ SafeAI lab-Carnegie Mellon University
"""
import numpy as np
import matplotlib.pyplot as plt
import os


class resp_states():
	def __init__(self,vehicle,carla):
		self._vehicle=vehicle
		self._controller=carla.VehicleControl()
		loc=vehicle.get_transform()
		self._x=loc.location.x
		self._y=loc.location.y
		self._yaw=loc.rotation.yaw
		self._vx=vehicle.get_velocity().x
		self._vy=vehicle.get_velocity().y
		self._t=0
		self._dt=1/20
		self._posx=[]
		self._posy=[]
		self._velx=[]
		self._vely=[]
		self._oryaw=[]
		self._throttle=[]
		self._brake=[]
		self._steer=[]
		self._frame=0.0
		self._v_max=2  # maximum allowed velocity of the vehicle in m/s
		self._plotted=False
	
	def update_values(self,throttle_out,brake_out,steer_out,v_long,v_lat,yaw):
		vehicle=self._vehicle
		loc=vehicle.get_transform()
		self._x=loc.location.x
		self._y=loc.location.y
		self._yaw=loc.rotation.yaw
		self._vx=vehicle.get_velocity().x
		self._vy=vehicle.get_velocity().y
		self._frame+=1
		self._posx.append(self._x)
		self._posy.append(self._y)
		self._velx.append(v_long)
		self._vely.append(v_lat)
		self._oryaw.append(np.pi*self._yaw/180)
		self._throttle.append(throttle_out)
		self._brake.append(brake_out)
		self._steer.append(steer_out)
		self._t+=self._dt

	def update_controls(self):
		vx=self._vx
		vy=-self._vy
		vehicle=self._vehicle
		yaw=-np.pi*self._yaw/180
		v_long=vy*np.sin(yaw)+vx*np.cos(yaw)
		v_lat=vy*np.cos(yaw)-vx*np.sin(yaw)
		# ---Bang Bang Longitudanal Control-----
		if np.linalg.norm(np.array([vx,vy]))<self._v_max:
			throttle_out=0.4
			brake_out=0.0
		else:
			throttle_out=0.0
			brake_out=0.0
		

		# --Lateral Control-------
		freq=1   # frequency in hz
		steer_out=0.3*np.sin(2*np.pi*freq*self._t)
		# steer_out=0.1
		# ----Sending and applying the control values-----
		self._controller.throttle=throttle_out
		self._controller.brake=brake_out
		self._controller.steer=steer_out

		vehicle.apply_control(self._controller)

		self.update_values(throttle_out,brake_out,steer_out,v_long,v_lat,yaw)
		print('time:---------',self._t)
		if self._t>70 and self._plotted==False:
			Statesave=np.transpose(np.array([self._posx,self._posy,self._velx,self._vely,self._oryaw,self._steer]))
			np.save('Figures/Cst3_1_hz.npy',Statesave)
			# self.plot_graphs()
			self._plotted=True
	
	def plot_graphs(self):
		'''
		Function to plot graphs for the states of the vehicle
		'''
		fig,axes=plt.subplots(3,2,figsize=(32,20))
		#  plotting trajectory of the vehicle
		axes[0,0].plot(self._posx,self._posy)
		axes[0,0].set_title('Trajectory of vehcile (m)')
		axes[0,0].set_ylabel('Y')
		axes[0,0].set_xlabel('X')
		axes[0,0].set_aspect(aspect=1.0)
		
		# plotting vx
		axes[1,0].plot(self._velx)
		axes[1,0].set_title('Vx of the vehicle (m/s)')
		axes[1,0].set_ylabel('Vx')
		axes[1,0].set_xlabel('Time')

		# plotting vy
		axes[2,0].plot(self._vely)
		axes[2,0].set_title('Vy of the vehicle (m/s)')
		axes[2,0].set_ylabel('Vy')
		axes[2,0].set_xlabel('Time')
 
		# plotting throttle
		axes[0,1].plot(self._throttle)
		axes[0,1].set_title('Throttle Command [0,1]')
		axes[0,1].set_ylabel('Throttle')
		axes[0,1].set_xlabel('Time')

		#  PLotting Steering Command
		axes[1,1].plot(self._steer)
		axes[1,1].set_title('Steering Commmand [-1,1] (freq_0.5hzs')
		axes[1,1].set_ylabel('Steer')
		axes[1,1].set_xlabel('Time')

		#  Plotting the vehcile yaw
		axes[2,1].plot(self._oryaw)
		axes[2,1].set_title('Vehicle Yaw (degress)')
		axes[2,1].set_ylabel('Vehicle Yaw')
		axes[2,1].set_xlabel('Time')

		fig.savefig('Figures/results_freq_0_5hz.png')


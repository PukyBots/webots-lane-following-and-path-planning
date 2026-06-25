from controller import Robot

def run_robot(robot):
    time = 32
    max_v = 6.28
    
    left_m = robot.getDevice('left wheel motor')
    right_m = robot.getDevice('right wheel motor')
    
    left_m.setPosition(float('inf'))
    right_m.setPosition(float('inf'))
    
    left_ir = robot.getDevice('IR_2')
    right_ir = robot.getDevice('IR_1')
    left_ir.enable(time)
    right_ir.enable(time)
    
    base = 5.5
    Kp = 0.085
       
    prev_error = 0
    Kd = 0.01
    
    int = 0
    Ki = 0.005
    

    
    while robot.step(time) !=-1:
        
    
        left = left_ir.getValue()
        right = right_ir.getValue()
        
        error = left - right
        d = error - prev_error
        
        int = 0.9*int+error
        if int > 100:
            int = 100
        if int < -100:
            int = -100
        
        change = error*Kp  + d*Kd + Ki*int
  
       
        
        left_speed = base-change
        right_speed = base+change
       # print(left_speed, " - ", right_speed)
        
        
      #  if right_speed > 6.28:
       #     overflow = right_speed - 6.28
     #       right_speed = 6.28
     #       left_speed -= overflow

      #  if left_speed > 6.28:
       #     overflow = left_speed - 6.28
        #    left_speed = 6.28
         #   right_speed -= overflow
         
         
         
#        if left_speed > 6.28:
 #           left_speed = 6.28
#
 #       if right_speed > 6.28:
  #          right_speed = 6.28
   #         
    #    if left_speed < -6.28:
     #       left_speed = -6.28
#
 #       if right_speed < -6.28:
  #          right_speed = -6.28
  
  
        if abs(right_speed)>6.27 or abs(left_speed)>6.27:
            a = max(abs(left_speed), abs(right_speed))
            scale = 6.27/a
            left_speed *= scale
            right_speed *= scale
        
            
        left_m.setVelocity(left_speed)
        right_m.setVelocity(right_speed)
        
        prev_error = error
        
        
        
        
#        left_m.setVelocity(6.28)
 #       right_m.setVelocity(6.28)
  #  
   #     if left>bw_range and right<bw_range:
    #        left_m.setVelocity(-1)
     #       right_m.setVelocity(6.28)
      #      
       # if left<bw_range and right>bw_range:
        #    left_m.setVelocity(6.28)
         #   right_m.setVelocity(-1)
        
       # print(left_ir.getValue(), " - ", right_ir.getValue())
    

if __name__ == "__main__":

    bot = Robot()
    run_robot(bot)



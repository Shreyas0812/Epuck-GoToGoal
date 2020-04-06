vrep = remApi('remoteApi');
vrep.simxFinish(-1);

clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
    
if (clientID>-1)
    disp('Connected');
    
    %Setting Constant linear velocity
    k = 0.2;
    
    %Handle------------------------------------------------------------------------------------------------
    [returnCode,left_Motor]=vrep.simxGetObjectHandle(clientID,'ePuck_leftJoint',vrep.simx_opmode_blocking);
    [returnCode,right_Motor]=vrep.simxGetObjectHandle(clientID,'ePuck_rightJoint',vrep.simx_opmode_blocking);
      
    [returnCode,Start]=vrep.simxGetObjectHandle(clientID,'DummyStart',vrep.simx_opmode_blocking);
    [returnCode,End]=vrep.simxGetObjectHandle(clientID,'DummyEnd',vrep.simx_opmode_blocking);
      
    [returnCode,EP]=vrep.simxGetObjectHandle(clientID,'ePuck',vrep.simx_opmode_blocking);
    
    %code--------------------------------------------------------------------------------------------------
    
    L=0.07;   %Diameter of ePuck
    D=0.041;  %Diameter of wheel
    
    %Stopping at the start position
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,0,vrep.simx_opmode_blocking);
    pause(2);
    
    %Loop
    for (i=1:100)
    
    %Obtaining angle
    [returnCode,A]=vrep.simxGetObjectPosition(clientID,EP,-1,vrep.simx_opmode_blocking);
    [returnCode,B]=vrep.simxGetObjectPosition(clientID,End,-1,vrep.simx_opmode_blocking);
    y21 = B(2) - A(2);
    x21 = B(1) - A(1);
    
    xsq = x21^2;
    ysq = y21^2;
    
    d = sqrt(xsq+ysq)
    
    %proportional control
    v=k*d;
    %v = 2*v;
    angle = atan(y21/x21);
    w = angle;
  
    %Checking Orientation
    [returnCode,C]=vrep.simxGetObjectOrientation(clientID,EP,-1,vrep.simx_opmode_blocking);
    a = C(1);
    b = C(2);
    
    if (a+b < 0.03) 
        disp('here');
        w = 0.01;
    end
    
    %Setting Angular Velocity  (Unicycle Model)
    w = w*L;
    vr = (v+w)/D;
    vl = (v-w)/D;
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,vl,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,vr,vrep.simx_opmode_blocking);
    
   
    pause(0.05);
           
        %Checking if ePuck is close to the target and stop accordingly
        [returnCode,A]=vrep.simxGetObjectPosition(clientID,EP,-1,vrep.simx_opmode_blocking);
        [returnCode,B]=vrep.simxGetObjectPosition(clientID,End,-1,vrep.simx_opmode_blocking);
        y21 = B(2) - A(2);
        x21 = B(1) - A(1);
        xsq = x21^2;
        ysq = y21^2;
    
        d = sqrt(xsq+ysq)
        if (d<0.08)
            disp('breaking');
            break;
        end
        
    pause(0.05);
    end
    
    %Stopping the ePuck
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,0,vrep.simx_opmode_blocking);
    
    disp('Done');
end
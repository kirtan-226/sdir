--------------------Content of the UI--------------------

--[[
Save the values which are written in the left handside Editfields.
Input:
    ui = UI Handler Value
    id = as Number of the Editfield
    newValue = Content of the Editfield
--]]
function editcart(ui, id, newValue)
    edit_ids[id-999] = id
    editvalues[id] = newValue
end

--[[
Save the values which are written in the right handside Editfields.
Input: 
    ui = Handler Value
    id = as Number of the Editfield
    newValue = Content of the Editfield
--]]
function editjp(ui, id, newValue)
    jpedit_ids[id-1999] = id
    jpeditvalues[id] = newValue
end

--[[
Setting up the Targetcoordinatesystem.
Building a JSON-String from the Input.
Sending the JSON-String to Backend via String Signal.
Receiving a String-Signal from the Backend.
Decoding of that JSON-String and Ouput to the Editfields.
Input:
    ui = UI Handler Value
    id = as Number of the Button
--]]
function applyDummy(ui,_)
    local comcnt = simUI.getComboboxItemCount(ui, 1013)
    local m_a = 0
    local m_b = 0
    local m_c = 0
    for _=1,comcnt do
        simUI.removeComboboxItem(ui,1013,1)
    end
    tip_pos = sim.getObjectPosition(tip,robot)
    tip_ori = sim.getObjectOrientation(tip,robot)
    local js
    local str
    if (simUI.getRadiobuttonValue(ui,1015)==1) then
        for key, _ in pairs(editvalues) do
            editvalues[key] = simUI.getEditValue(ui, key)
        end
        local new_pos = {}
        for i= 1,3 do
            if (not edit_ids[i]) then
                new_pos[i] = 0
            else
                new_pos[i] = editvalues[edit_ids[i]]
            end
        end
        local new_ori = {}
        for i= 1,3 do
            if (not edit_ids[i+3]) then
                new_ori[i] = 0
            else
                new_ori[i] = math.rad(editvalues[edit_ids[i+3]])
            end
        end
        sim.setObjectPosition(ik_target, robot, new_pos)
        set_orientation(ik_target, robot, new_ori)
        js = {
            op = 1,
            data = {{ 
               m_a = tonumber(new_ori[1]),
               m_b = tonumber(new_ori[2]),
               m_c = tonumber(new_ori[3]),
               m_x = tonumber(new_pos[1]),
               m_y = tonumber(new_pos[2]),
               m_z = tonumber(new_pos[3])
            }}
        }
        --print("Value of m_a: " .. m_a)
        --print("Value of m_b: " .. m_b)
        --print("Value of m_c: " .. m_c)
        str = json.encode (js, { indent = true })
        sim.setStringSignal("callsignal",str)
    elseif (simUI.getRadiobuttonValue(ui,1016)==1) then
        for key, _ in pairs(jpeditvalues) do
            jpeditvalues[key] = simUI.getEditValue(ui, key)
        end
        local new_jp = {}
        for i= 1,6 do
            if (not jpedit_ids[i]) then
                new_jp[i] = 0
            else
                new_jp[i] = jpeditvalues[jpedit_ids[i]]
            end
        end
        for i = 1,6 do
            if simUI.getRadiobuttonValue(ui, 2007)==1 then
                new_jp[i] = math.rad(new_jp[i])
            end
        end
        setConfigEndPoint(ui,new_jp)
        js = {
            op = 0,
            data = {{ 
               j0 = tonumber(new_jp[1]),
               j1 = tonumber(new_jp[2]),
               j2 = tonumber(new_jp[3]),
               j3 = tonumber(new_jp[4]),
               j4 = tonumber(new_jp[5]),
               j5 = tonumber(new_jp[6])
            }}
        }
        str = json.encode (js, { indent = true })
        sim.setStringSignal("callsignal",str)
    end
end

--[[
Receiving a String-Signal from the Backend.
Decoding of the recieved JSON-String and Output to the Editfields.
This function is called by the controller.
--]]
function returnSignal()
    local ret = sim.getStringSignal("returnsignal")
    local ui = ui_1
    if ret then
        local obj, _, err = json.decode(ret, 1, nil)
        if err then
           sim.addStatusbarMessage("Error: " .. err)  
        else
            if obj.op == 0 then
                if simUI.getRadiobuttonValue(ui, 2007) == 1 then
                    simUI.setEditValue(ui, 2000, tostring(math.deg(obj.data[1].j0)))
                    simUI.setEditValue(ui, 2001, tostring(math.deg(obj.data[1].j1)))
                    simUI.setEditValue(ui, 2002, tostring(math.deg(obj.data[1].j2)))
                    simUI.setEditValue(ui, 2003, tostring(math.deg(obj.data[1].j3)))
                    simUI.setEditValue(ui, 2004, tostring(math.deg(obj.data[1].j4)))
                    simUI.setEditValue(ui, 2005, tostring(math.deg(obj.data[1].j5)))
                else
                    simUI.setEditValue(ui, 2000, tostring(obj.data[1].j0))
                    simUI.setEditValue(ui, 2001, tostring(obj.data[1].j1))
                    simUI.setEditValue(ui, 2002, tostring(obj.data[1].j2))
                    simUI.setEditValue(ui, 2003, tostring(obj.data[1].j3))
                    simUI.setEditValue(ui, 2004, tostring(obj.data[1].j4))
                    simUI.setEditValue(ui, 2005, tostring(obj.data[1].j5))
                end
                for i = 1, #obj.data do
                    cconf[i] = {obj.data[i].j0, obj.data[i].j1, obj.data[i].j2, obj.data[i].j3, obj.data[i].j4, obj.data[i].j5}
                    simUI.insertComboboxItem(ui, 1013, i, i .. ". Configuration")
                end
            elseif obj.op == 1 then
                -- Make sure obj.data is a table (array)
                if type(obj.data) == "table" then
                    -- Access the first element in the table (index 1)
                    local data1 = obj.data[1]
        
                    if data1 then
                        simUI.setEditValue(ui, 1000, tostring(data1.m_x))
                        simUI.setEditValue(ui, 1001, tostring(data1.m_y))
                        simUI.setEditValue(ui, 1002, tostring(data1.m_z))
                        simUI.setEditValue(ui, 1003, tostring(math.deg(data1.m_a)))
                        simUI.setEditValue(ui, 1004, tostring(math.deg(data1.m_b)))
                        simUI.setEditValue(ui, 1005, tostring(math.deg(data1.m_c)))
                    else
                        print("data1 is nil.")
                    end
                else
                    print("obj.data is not a table.")
                end
            end
        end
    else
        print("No Return-Signal. Try again or Restart the Programm!")
    end
    changeEnabled(ui, true)
    return {}, {}, {}, ''
end

--[[
Assisting function to set some Parts of the GUI enabled.
Input:
    enabled = Boolean 
--]]
function changeEnabled(ui,enabled)
    simUI.setEnabled(ui,1007,enabled)
    simUI.setEnabled(ui,1008,enabled)
    simUI.setEnabled(ui,1009,enabled)
    simUI.setEnabled(ui,1010,enabled)
    simUI.setEnabled(ui,1011,enabled)
end

--[[
Assisting function to set the Targetcoordinatesystem with Jointconfigurations as Input.
Input:
    c = configuration
--]]
function setConfigEndPoint(ui,c)
    local sc = {}
    for i=1,#joints do
        sc[i] = sim.getJointPosition(jh[i]) 
    end
    for i=1,#joints do
        sim.setJointPosition(jh[i], c[i])
    end
    local tip_pos = sim.getObjectPosition(tip,-1)
    local tip_ori = sim.getObjectOrientation(tip,-1)
    sim.setObjectPosition(ik_dummy,-1,tip_pos)
    sim.setObjectOrientation(ik_dummy,-1,tip_ori)
    sim.setObjectPosition(ik_target,-1,tip_pos)
    sim.setObjectOrientation(ik_target,-1,tip_ori)
    local new_pos = sim.getObjectPosition(tip,robot)
    --local new_ori = sim.getObjectOrientation(tip,-1)
    local new_ori = get_orientation(tip, robot)
    for i=1,6 do
        if i <=3 then
            simUI.setEditValue(ui,999+i,tostring(math.round(new_pos[i],3)))
        else
            simUI.setEditValue(ui,999+i,tostring(math.round(math.deg(new_ori[i-3]),3)))
        end
    end
    for i=1,#joints do
        sim.setJointPosition(jh[i], sc[i])
    end
end

--[[
Button to send the Start- and the Targetconfiguration of the Manipulator to
the Backend, where the Calculations should be done.
Setting up a JSON-String with the typ of Movement, the Start- and the Endconfiguration.
Input:
    ui = UI Handler Value
    id = as Number of the Button
--]]
function CalculateIK(ui,id)
    local new_pos = sim.getObjectPosition(ik_target,robot)
    local new_ori = sim.getObjectOrientation(ik_target, robot)
    local our_ori = get_orientation(ik_target, robot)
    
    sim.setObjectPosition(ik_target, robot, new_pos)
    sim.setObjectOrientation(ik_target, robot, new_ori)

    
    for i=1,6 do
        if i <=3 then
            simUI.setEditValue(ui,999+i,tostring(math.round(new_pos[i],3)))
        else
            simUI.setEditValue(ui,999+i,tostring(math.round(math.deg(new_ori[i-3]),3)))
        end
    end
    
    local curr_c = {}
    local tar_c = {}
    local js
    for i=1,#joints do
        curr_c[i] = sim.getJointPosition(jh[i]) 
    end
    for i=1,#joints do
        if simUI.getRadiobuttonValue(ui, 2007)==1 then
            tar_c[i] = math.rad(simUI.getEditValue(ui,i+1999))
        else
            tar_c[i] = simUI.getEditValue(ui,i+1999)
        end
    end
    if (simUI.getRadiobuttonValue(ui,1007) == 1) then
        if (simUI.getRadiobuttonValue(ui,1010) == 1) then
            js = {
                op = 2,
                data = {{ 
                   j0 = tonumber(curr_c[1]),
                   j1 = tonumber(curr_c[2]),
                   j2 = tonumber(curr_c[3]),
                   j3 = tonumber(curr_c[4]),
                   j4 = tonumber(curr_c[5]),
                   j5 = tonumber(curr_c[6])
                },
                { 
                   j0 = tonumber(tar_c[1]),
                   j1 = tonumber(tar_c[2]),
                   j2 = tonumber(tar_c[3]),
                   j3 = tonumber(tar_c[4]),
                   j4 = tonumber(tar_c[5]),
                   j5 = tonumber(tar_c[6])
                }}
            }
        elseif (simUI.getRadiobuttonValue(ui,1009) == 1) then
            js = {
                op = 3,
                data = {{ 
                   j0 = tonumber(curr_c[1]),
                   j1 = tonumber(curr_c[2]),
                   j2 = tonumber(curr_c[3]),
                   j3 = tonumber(curr_c[4]),
                   j4 = tonumber(curr_c[5]),
                   j5 = tonumber(curr_c[6])
                },
                { 
                   j0 = tonumber(tar_c[1]),
                   j1 = tonumber(tar_c[2]),
                   j2 = tonumber(tar_c[3]),
                   j3 = tonumber(tar_c[4]),
                   j4 = tonumber(tar_c[5]),
                   j5 = tonumber(tar_c[6])
                }}
            }
        end
    elseif (simUI.getRadiobuttonValue(ui,1008) == 1) then
        js = {
            op = 4,
            data = {{ 
               j0 = tonumber(curr_c[1]),
               j1 = tonumber(curr_c[2]),
               j2 = tonumber(curr_c[3]),
               j3 = tonumber(curr_c[4]),
               j4 = tonumber(curr_c[5]),
               j5 = tonumber(curr_c[6])
            },
            { 
               j0 = tonumber(tar_c[1]),
               j1 = tonumber(tar_c[2]),
               j2 = tonumber(tar_c[3]),
               j3 = tonumber(tar_c[4]),
               j4 = tonumber(tar_c[5]),
               j5 = tonumber(tar_c[6])
            }}
        }
    end
    local str = json.encode (js, { indent = true })
    sim.setStringSignal("callsignal",str)
end

--[[
Change the Movement Mode and allow to use sync or async Movement.
Input:
    ui = UI Handler Value
    id = as Number of the Button
--]]
function switchMVMode(ui,id)
    if (id==1007) then
        simUI.setEnabled(ui,1009,true)
        simUI.setEnabled(ui,1010,true)
        simUI.setRadiobuttonValue(ui,1009,1)
    else
        simUI.setEnabled(ui,1009,false)
        simUI.setEnabled(ui,1010,false)
    end
end

--[[
Use that funtion to switch between the different Configurations that where calcutated for
a target point in the Backend.
Input:
    ui = UI Handler Value
    id = as Number of the Combobox
    newValue = Content of the Combobox
--]]
function switchConfig(ui,_,newValue)
    if (newValue ~= 0) then
        for i=1,6 do
            if (simUI.getRadiobuttonValue(ui,2007)==1) then
                simUI.setEditValue(ui,1999+i,tostring(math.deg(cconf[newValue][i])))
                editjp(ui,1999+i,tostring(math.deg(cconf[newValue][i])))
            else
                simUI.setEditValue(ui,1999+i,tostring(cconf[newValue][i]))
                editjp(ui,1999+i,tostring(cconf[newValue][i]))
            end
        end
        setConfigEndPoint(ui,cconf[newValue])
    end
end

--[[
Change between Cartisian Mode or Joint Configuartion Mode.
Input:
    ui = UI Handler Value
    id = as Number of the Button
--]]
function switchOPMode(ui,id)
    if (id==1015) then
        simUI.setEnabled(ui,1017,true)
        simUI.setEnabled(ui,1018,false)
    else
        simUI.setEnabled(ui,1017,false)
        simUI.setEnabled(ui,1018,true)
    end
end

--[[
Change between Radians and Degrees in the right handside Editfields.
Input:
    ui = UI Handler Value
    id = as Number of the Button
--]]
function radiobuttonClick(ui,id)
    if (id==2007) then
        if not degree then
            for i=1,6 do
                local value = simUI.getEditValue(ui,i+1999)
                simUI.setEditValue(ui,i+1999,tostring(math.deg(value)))
            end
            degree = true
        end
    else
        if degree then
            for i=1,6 do
                local value = simUI.getEditValue(ui,i+1999)
                simUI.setEditValue(ui,i+1999,tostring(math.rad(value)))
            end
            degree = false
        end
    end
end

---------------------------------------------

--Close the UI
function closeEventHandler(h)
    simUI.hide(h)
end

--Close the second UI by hitting the Button
function buttonok(ui,id)
    simUI.hide(ui_2)
    simUI.show(ui_1)
end

--Function to round Values
function math.round(number, decimals, method)
    decimals = decimals or 0
    local factor = 10 ^ decimals
    if (method == "ceil" or method == "floor") then return math[method](number * factor) / factor
    else return tonumber(("%."..decimals.."f"):format(number)) end
end

--[[
This function calculates the Euler angles according to your convention
Input:
    handle      = handle of the object
    relative    = objective to calculate orientation relative to, or -1 for absolute
--]]
function get_orientation(handle, relative)
    mat = sim.getObjectMatrix(handle, relative)
    local a, b, c
    new_angles = sim.getEulerAnglesFromMatrix(mat)
    a = new_angles[1]
    b = new_angles[2]
    c = new_angles[3]
    return {a, b, c}
end

--[[
This function calculates sets orientation from Euler angles according to your convention
Input:
    handle      = handle of the object
    relative    = objective to calculate orientation relative to, or -1 for absolute
--]]
function set_orientation(handle, relative, angles)
    mat = sim.buildIdentityMatrix()
    mat = sim.rotateAroundAxis(mat, {0, 0, 1}, {0, 0, 0}, angles[1])-- about z with a
    mat = sim.rotateAroundAxis(mat, {0, 1, 0}, {0, 0, 0}, angles[2])-- about y with b
    mat = sim.rotateAroundAxis(mat, {1, 0, 0}, {0, 0, 0}, angles[3])-- about x with c
    
    new_angles = sim.getEulerAnglesFromMatrix(mat)
    sim.setObjectOrientation(handle, relative, new_angles)
end

--Initialize the UI
function sysCall_init()
    sim=require'sim'
    simUI=require'simUI'
	
    coord_dialog = [[<ui closeable="true" on-close="closeEventHandler" layout="vbox" title="Coordinates" resizable="true">
    <label text="Choose Operation Mode for the Robot Manipulator."></label>
    <group layout="hbox">
        <radiobutton text="By Cartisian Coordinates" checked="true" on-click="switchOPMode" id="1015" />
        <radiobutton text="By Joint Angles" on-click="switchOPMode" id="1016" />
    </group>
    <group layout="hbox">
        <group layout="vbox" id="1017">
            <label text="Please enter coordinates(in Meter) and orientation(in Degree)."></label>
            <group layout="hbox">
                <group>
                    <label text="X:"></label>
                    <edit id="1000" value="0" on-editing-finished="editcart"></edit>
                </group>
                <group>
                    <label text="Y:"></label>
                    <edit id="1001" value="0" on-editing-finished="editcart"></edit>
                </group>
                <group>
                    <label text="Z:"></label>
                    <edit id="1002" value="0" on-editing-finished="editcart"></edit>
                </group>
            </group>
            <group layout="hbox">
                <group>
                    <label text="A:"></label>
                    <edit id="1003" value="0" on-editing-finished="editcart"></edit>
                </group>
                <group>
                    <label text="B:"></label>
                    <edit id="1004" value="0" on-editing-finished="editcart"></edit>
                </group>
                <group>
                    <label text="C:"></label>
                    <edit id="1005" value="0" on-editing-finished="editcart"></edit>
                </group>
            </group>
        </group>
        <group layout="vbox">
            <label text="Please enter the Joint Angles you want to change. You could either use Radian or Degree as Input."></label>
            <group layout="hbox">
                <radiobutton text="Degree" on-click="radiobuttonClick" id="2007" />
                <radiobutton text="Radian" on-click="radiobuttonClick" id="2008" />
            </group>
            <group layout="vbox" enabled="false" id="1018">
                <group layout="hbox">
                    <group layout="vbox">
                        <label text="Theta 1:"></label>
                        <edit id="2000" value="0" on-editing-finished="editjp"></edit>
                    </group>
                    <group layout="vbox">
                        <label text="Theta 2:"></label>
                        <edit id="2001" value="0" on-editing-finished="editjp"></edit>
                    </group>
                    <group layout="vbox">
                        <label text="Theta 3:"></label>
                        <edit id="2002" value="0" on-editing-finished="editjp"></edit>
                    </group>
                </group>
                <group layout="hbox">
                    <group layout="vbox">
                        <label text="Theta 4:"></label>
                        <edit id="2003" value="0" on-editing-finished="editjp"></edit>
                    </group>
                    <group layout="vbox">
                        <label text="Theta 5:"></label>
                        <edit id="2004" value="0" on-editing-finished="editjp"></edit>
                    </group>
                    <group layout="vbox">
                        <label text="Theta 6:"></label>
                        <edit id="2005" value="0" on-editing-finished="editjp"></edit>
                    </group>
                </group>
            </group>
            <label text="Please select the Configuration of the Endeffector."></label>
            <combobox id="1013" on-change="switchConfig"></combobox>
        </group>
    </group>
    <group layout="vbox">
        <button text="Apply" id="1006" on-click="applyDummy"></button>
        <label text="Please select an operation mode."></label>
        <group layout="hbox">
            <radiobutton text="PTP" enabled="false" checked="true" on-click="switchMVMode" id="1007" />
            <radiobutton text="LIN" enabled="false" on-click="switchMVMode" id="1008" />
        </group>
        <group layout="hbox">
            <radiobutton text="sync" checked="true" enabled="false" on-click="" id="1009" />
            <radiobutton text="async" enabled="false" on-click="" id="1010" />
        </group>
        <button text="Calculate and Move" id="1011" enabled="false" on-click="CalculateIK"></button>
    </group>
</ui>]]
    
    error = [[<ui closeable="false" on-close="buttonok" layout="vbox" title="Error">
	<label text="No path is available for the entered Values."></label>
	<button text="OK" on-click="buttonok"></button>
	</ui>]]
    
    edit_ids = {}
    editvalues = {}
    jpedit_ids = {}
    jpeditvalues = {}
    c = {}
    cconf={}
    degree = false

    json=require("dkjson")

    ui_1=simUI.create(coord_dialog)
    ui_2=simUI.create(error)

    local myuis = {ui_1,ui_2}
    sim.setStringSignal("uisignal",sim.packTable(myuis))

    ik_dummy = sim.getObject('/ik_target')
    ik_target = sim.getObject('/testTarget1')
    tip = sim.getObject('/KR120_2700_2_tip')
    robot = sim.getObject('/Graph')
    joints = {  'KR120_2700_2_joint1','KR120_2700_2_joint2',
                'KR120_2700_2_joint3','KR120_2700_2_joint4',
                'KR120_2700_2_joint5','KR120_2700_2_joint6'}
    jh={-1,-1,-1,-1,-1,-1}
    jp={-1,-1,-1,-1,-1,-1}
    for i=1,#joints do
        jh[i]=sim.getObject("/KR120_2700_2_joint" .. i)
        jp[i]=sim.getJointPosition(jh[i])
    end
    tip_pos = sim.getObjectPosition(tip,robot)
    tip_ori = sim.getObjectOrientation(tip,robot)
    sim.setObjectPosition(ik_dummy, robot, tip_pos)
    sim.setObjectOrientation(ik_dummy, robot, tip_ori)
    sim.setObjectPosition(ik_target, robot, tip_pos)
    sim.setObjectOrientation(ik_target, robot, tip_ori)
    simUI.hide(ui_2)
    simUI.setRadiobuttonValue(ui_1,2008,1)

    for i=1,6 do
        if i <=3 then
            simUI.setEditValue(ui_1,999+i,tostring(math.round(tip_pos[i],3)))
            editcart(ui_1,999+i,tostring(math.round(tip_pos[i],3)))
        else
            simUI.setEditValue(ui_1,999+i,tostring(math.round(math.deg(tip_ori[i-3]),3)))
            editcart(ui_1,999+i,tostring(math.round(math.deg(tip_ori[i-3]),3)))
        end
    end
    for i=1,6 do
        simUI.setEditValue(ui_1,1999+i,tostring(math.round(jp[i],3)))
        editjp(ui_1,1999+i,tostring(math.round(jp[i],3)))
    end
    simUI.insertComboboxItem(ui_1,1013,0,"Select a config:")
end

--Close all UI's at the end of the simulation
function sysCall_cleanup()
    simUI.destroy(ui_1)
    simUI.destroy(ui_2)
end
classdef CustomMsgConsts
    %CustomMsgConsts This class stores all message types
    %   The message types are constant properties, which in turn resolve
    %   to the strings of the actual types.
    
    %   Copyright 2014-2017 The MathWorks, Inc.
    
    properties (Constant)
        quadrotor_msgs_FlatOutputs = 'quadrotor_msgs/FlatOutputs'
        std_srvs_SetBool = 'std_srvs/SetBool'
        std_srvs_SetBoolRequest = 'std_srvs/SetBoolRequest'
        std_srvs_SetBoolResponse = 'std_srvs/SetBoolResponse'
        std_srvs_Trigger = 'std_srvs/Trigger'
        std_srvs_TriggerRequest = 'std_srvs/TriggerRequest'
        std_srvs_TriggerResponse = 'std_srvs/TriggerResponse'
        trackers_manager_Transition = 'trackers_manager/Transition'
        trackers_manager_TransitionRequest = 'trackers_manager/TransitionRequest'
        trackers_manager_TransitionResponse = 'trackers_manager/TransitionResponse'
    end
    
    methods (Static, Hidden)
        function messageList = getMessageList
            %getMessageList Generate a cell array with all message types.
            %   The list will be sorted alphabetically.
            
            persistent msgList
            if isempty(msgList)
                msgList = cell(7, 1);
                msgList{1} = 'quadrotor_msgs/FlatOutputs';
                msgList{2} = 'std_srvs/SetBoolRequest';
                msgList{3} = 'std_srvs/SetBoolResponse';
                msgList{4} = 'std_srvs/TriggerRequest';
                msgList{5} = 'std_srvs/TriggerResponse';
                msgList{6} = 'trackers_manager/TransitionRequest';
                msgList{7} = 'trackers_manager/TransitionResponse';
            end
            
            messageList = msgList;
        end
        
        function serviceList = getServiceList
            %getServiceList Generate a cell array with all service types.
            %   The list will be sorted alphabetically.
            
            persistent svcList
            if isempty(svcList)
                svcList = cell(3, 1);
                svcList{1} = 'std_srvs/SetBool';
                svcList{2} = 'std_srvs/Trigger';
                svcList{3} = 'trackers_manager/Transition';
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            serviceList = svcList;
        end
        
        function actionList = getActionList
            %getActionList Generate a cell array with all action types.
            %   The list will be sorted alphabetically.
            
            persistent actList
            if isempty(actList)
                actList = cell(0, 1);
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            actionList = actList;
        end
    end
end

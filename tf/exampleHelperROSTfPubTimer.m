function exampleHelperROSTfPubTimer(~, ~, tfpub)
%exampleHelperROSTfPubTimer - Timer update function called by tf example
%
%   See also exampleHelperROSStartTfPublisher.

%   Copyright 2014-2015 The MathWorks, Inc.

persistent tfmsg1 tfmsg2

if isempty(tfmsg1)
    tfmsg1 = newTF2Message('robot_base', 'mounting_point', [1.0 0 0], [1 0 0 0]);
    tfmsg2 = newTF2Message('mounting_point', 'camera_center', [0 0 0.5], [sqrt(2)/2 0 sqrt(2)/2 0]);
end

% Set current time stamp
% Only do this, if the global node is active. Otherwise, calls to rostime
% might fail.
if ros.internal.Global.isNodeActive
    currentTime = rostime('now');
    tfmsg1.Transforms(1).Header.Stamp = currentTime;
    tfmsg2.Transforms(1).Header.Stamp = currentTime;
end

% Publish the transformations to /tf
if isvalid(tfpub)
    send(tfpub, tfmsg1);
    send(tfpub, tfmsg2);
end

end

function tf2Msg = newTF2Message(targetFrame, sourceFrame, translation, rotation)
%newTF2Message Create a new tf2_msgs/TFMessage with one transform

tfStampedMsg = newTransformStamped(...
        targetFrame, sourceFrame, translation, rotation);

tf2Msg = rosmessage('tf2_msgs/TFMessage');
tf2Msg.Transforms = tfStampedMsg;
end

function tfStampedMsg = newTransformStamped(targetFrame, sourceFrame, translation, rotation)
%newTransformStamped Create a new geometry_msgs/TransformStamped message

tfStampedMsg = rosmessage('geometry_msgs/TransformStamped');
tfStampedMsg.ChildFrameId = sourceFrame;
tfStampedMsg.Header.FrameId = targetFrame;
if ros.internal.Global.isNodeActive
    tfStampedMsg.Header.Stamp = rostime('now');
end

tfStampedMsg.Transform.Translation.X = translation(1);
tfStampedMsg.Transform.Translation.Y = translation(2);
tfStampedMsg.Transform.Translation.Z = translation(3);

tfStampedMsg.Transform.Rotation.W = rotation(1);
tfStampedMsg.Transform.Rotation.X = rotation(2);
tfStampedMsg.Transform.Rotation.Y = rotation(3);
tfStampedMsg.Transform.Rotation.Z = rotation(4);
end
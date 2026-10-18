%exampleHelperROSStartTfPublisher Launch background publisher of Tf data
%   See also exampleHelperROSStopTfPublisher.

%   Copyright 2014-2015 The MathWorks, Inc.

% Create tf topic for publishing transformations
tfpub = rospublisher('/tf', 'tf2_msgs/TFMessage');

% Create a timer for publishing tf messages
tfpubtimer = ExampleHelperROSTimer(0.1, {@exampleHelperROSTfPubTimer,tfpub});

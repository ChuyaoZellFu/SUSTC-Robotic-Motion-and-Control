
"use strict";

let ApiOptions = require('./ApiOptions.js');
let KortexError = require('./KortexError.js');
let SubErrorCodes = require('./SubErrorCodes.js');
let ErrorCodes = require('./ErrorCodes.js');
let FrequencyResponse = require('./FrequencyResponse.js');
let ControlLoop = require('./ControlLoop.js');
let StepResponse = require('./StepResponse.js');
let CommandMode = require('./CommandMode.js');
let ActuatorConfig_SafetyLimitType = require('./ActuatorConfig_SafetyLimitType.js');
let ControlLoopParameters = require('./ControlLoopParameters.js');
let ActuatorConfig_ControlMode = require('./ActuatorConfig_ControlMode.js');
let LoopSelection = require('./LoopSelection.js');
let CustomDataIndex = require('./CustomDataIndex.js');
let CommandModeInformation = require('./CommandModeInformation.js');
let RampResponse = require('./RampResponse.js');
let SafetyIdentifierBankA = require('./SafetyIdentifierBankA.js');
let TorqueOffset = require('./TorqueOffset.js');
let CoggingFeedforwardMode = require('./CoggingFeedforwardMode.js');
let ControlLoopSelection = require('./ControlLoopSelection.js');
let AxisPosition = require('./AxisPosition.js');
let VectorDriveParameters = require('./VectorDriveParameters.js');
let AxisOffsets = require('./AxisOffsets.js');
let TorqueCalibration = require('./TorqueCalibration.js');
let EncoderDerivativeParameters = require('./EncoderDerivativeParameters.js');
let CoggingFeedforwardModeInformation = require('./CoggingFeedforwardModeInformation.js');
let ActuatorConfig_ControlModeInformation = require('./ActuatorConfig_ControlModeInformation.js');
let PositionCommand = require('./PositionCommand.js');
let ActuatorConfig_ServiceVersion = require('./ActuatorConfig_ServiceVersion.js');
let Servoing = require('./Servoing.js');
let CustomDataSelection = require('./CustomDataSelection.js');
let ActuatorCyclic_MessageId = require('./ActuatorCyclic_MessageId.js');
let StatusFlags = require('./StatusFlags.js');
let ActuatorCyclic_CustomData = require('./ActuatorCyclic_CustomData.js');
let CommandFlags = require('./CommandFlags.js');
let ActuatorCyclic_Command = require('./ActuatorCyclic_Command.js');
let ActuatorCyclic_ServiceVersion = require('./ActuatorCyclic_ServiceVersion.js');
let ActuatorCyclic_Feedback = require('./ActuatorCyclic_Feedback.js');
let WifiInformation = require('./WifiInformation.js');
let BridgeConfig = require('./BridgeConfig.js');
let PasswordChange = require('./PasswordChange.js');
let BridgeStatus = require('./BridgeStatus.js');
let PreComputedJointTrajectory = require('./PreComputedJointTrajectory.js');
let SystemTime = require('./SystemTime.js');
let BridgePortConfig = require('./BridgePortConfig.js');
let ActionNotification = require('./ActionNotification.js');
let UserNotification = require('./UserNotification.js');
let GripperCommand = require('./GripperCommand.js');
let Delay = require('./Delay.js');
let WifiConfigurationList = require('./WifiConfigurationList.js');
let SequenceTaskHandle = require('./SequenceTaskHandle.js');
let CartesianTrajectoryConstraint = require('./CartesianTrajectoryConstraint.js');
let ServoingModeNotification = require('./ServoingModeNotification.js');
let Base_RotationMatrix = require('./Base_RotationMatrix.js');
let TransformationMatrix = require('./TransformationMatrix.js');
let NetworkEvent = require('./NetworkEvent.js');
let SequenceTasks = require('./SequenceTasks.js');
let ConstrainedJointAngles = require('./ConstrainedJointAngles.js');
let Gripper = require('./Gripper.js');
let SignalQuality = require('./SignalQuality.js');
let SequenceTasksPair = require('./SequenceTasksPair.js');
let ServoingMode = require('./ServoingMode.js');
let IPv4Information = require('./IPv4Information.js');
let ActivateMapHandle = require('./ActivateMapHandle.js');
let Wrench = require('./Wrench.js');
let Admittance = require('./Admittance.js');
let WaypointValidationReport = require('./WaypointValidationReport.js');
let UserList = require('./UserList.js');
let ControllerInputType = require('./ControllerInputType.js');
let GripperRequest = require('./GripperRequest.js');
let MapGroupList = require('./MapGroupList.js');
let Base_SafetyIdentifier = require('./Base_SafetyIdentifier.js');
let SafetyNotificationList = require('./SafetyNotificationList.js');
let JointsLimitationsList = require('./JointsLimitationsList.js');
let CartesianLimitation = require('./CartesianLimitation.js');
let MappingHandle = require('./MappingHandle.js');
let MapGroup = require('./MapGroup.js');
let Base_ControlMode = require('./Base_ControlMode.js');
let WifiEnableState = require('./WifiEnableState.js');
let SequenceTasksConfiguration = require('./SequenceTasksConfiguration.js');
let Mapping = require('./Mapping.js');
let Base_Position = require('./Base_Position.js');
let ControllerNotification = require('./ControllerNotification.js');
let ActionList = require('./ActionList.js');
let Ssid = require('./Ssid.js');
let WifiInformationList = require('./WifiInformationList.js');
let JointTorque = require('./JointTorque.js');
let FirmwareBundleVersions = require('./FirmwareBundleVersions.js');
let Finger = require('./Finger.js');
let BridgeResult = require('./BridgeResult.js');
let WrenchMode = require('./WrenchMode.js');
let ControllerBehavior = require('./ControllerBehavior.js');
let NetworkNotificationList = require('./NetworkNotificationList.js');
let TrajectoryErrorReport = require('./TrajectoryErrorReport.js');
let Sequence = require('./Sequence.js');
let ShapeType = require('./ShapeType.js');
let ControllerHandle = require('./ControllerHandle.js');
let CommunicationInterfaceConfiguration = require('./CommunicationInterfaceConfiguration.js');
let MappingInfoNotification = require('./MappingInfoNotification.js');
let TrajectoryInfoType = require('./TrajectoryInfoType.js');
let ControllerList = require('./ControllerList.js');
let MapElement = require('./MapElement.js');
let Action = require('./Action.js');
let Faults = require('./Faults.js');
let UserEvent = require('./UserEvent.js');
let AdvancedSequenceHandle = require('./AdvancedSequenceHandle.js');
let ConstrainedPose = require('./ConstrainedPose.js');
let FullIPv4Configuration = require('./FullIPv4Configuration.js');
let FullUserProfile = require('./FullUserProfile.js');
let CartesianSpeed = require('./CartesianSpeed.js');
let ConstrainedOrientation = require('./ConstrainedOrientation.js');
let TwistLimitation = require('./TwistLimitation.js');
let ActionType = require('./ActionType.js');
let ConstrainedPosition = require('./ConstrainedPosition.js');
let OperatingModeNotificationList = require('./OperatingModeNotificationList.js');
let Base_ServiceVersion = require('./Base_ServiceVersion.js');
let BridgeIdentifier = require('./BridgeIdentifier.js');
let Base_GpioConfiguration = require('./Base_GpioConfiguration.js');
let JointTrajectoryConstraint = require('./JointTrajectoryConstraint.js');
let ActionNotificationList = require('./ActionNotificationList.js');
let SequenceTasksRange = require('./SequenceTasksRange.js');
let NavigationDirection = require('./NavigationDirection.js');
let RobotEventNotification = require('./RobotEventNotification.js');
let NetworkType = require('./NetworkType.js');
let OperatingModeInformation = require('./OperatingModeInformation.js');
let Base_CapSenseConfig = require('./Base_CapSenseConfig.js');
let GpioPinConfiguration = require('./GpioPinConfiguration.js');
let SequenceList = require('./SequenceList.js');
let BluetoothEnableState = require('./BluetoothEnableState.js');
let OperatingModeNotification = require('./OperatingModeNotification.js');
let Xbox360AnalogInputIdentifier = require('./Xbox360AnalogInputIdentifier.js');
let Waypoint = require('./Waypoint.js');
let ZoneShape = require('./ZoneShape.js');
let IPv4Configuration = require('./IPv4Configuration.js');
let TwistCommand = require('./TwistCommand.js');
let ProtectionZoneList = require('./ProtectionZoneList.js');
let WifiConfiguration = require('./WifiConfiguration.js');
let FirmwareComponentVersion = require('./FirmwareComponentVersion.js');
let ControllerElementHandle = require('./ControllerElementHandle.js');
let Waypoint_type_of_waypoint = require('./Waypoint_type_of_waypoint.js');
let UserProfile = require('./UserProfile.js');
let SequenceTaskConfiguration = require('./SequenceTaskConfiguration.js');
let Map = require('./Map.js');
let Point = require('./Point.js');
let ProtectionZoneEvent = require('./ProtectionZoneEvent.js');
let ConfigurationNotificationEvent = require('./ConfigurationNotificationEvent.js');
let Snapshot = require('./Snapshot.js');
let WrenchCommand = require('./WrenchCommand.js');
let JointNavigationDirection = require('./JointNavigationDirection.js');
let Base_Stop = require('./Base_Stop.js');
let ControllerConfigurationList = require('./ControllerConfigurationList.js');
let ControllerNotification_state = require('./ControllerNotification_state.js');
let ServoingModeInformation = require('./ServoingModeInformation.js');
let ControllerState = require('./ControllerState.js');
let NetworkHandle = require('./NetworkHandle.js');
let Base_JointSpeeds = require('./Base_JointSpeeds.js');
let ProtectionZoneNotification = require('./ProtectionZoneNotification.js');
let GpioEvent = require('./GpioEvent.js');
let GpioAction = require('./GpioAction.js');
let SoundType = require('./SoundType.js');
let SequenceInformation = require('./SequenceInformation.js');
let PreComputedJointTrajectoryElement = require('./PreComputedJointTrajectoryElement.js');
let ProtectionZoneHandle = require('./ProtectionZoneHandle.js');
let BackupEvent = require('./BackupEvent.js');
let Query = require('./Query.js');
let AppendActionInformation = require('./AppendActionInformation.js');
let ChangeTwist = require('./ChangeTwist.js');
let Twist = require('./Twist.js');
let ControllerNotificationList = require('./ControllerNotificationList.js');
let AdmittanceMode = require('./AdmittanceMode.js');
let CartesianLimitationList = require('./CartesianLimitationList.js');
let RFConfiguration = require('./RFConfiguration.js');
let BridgeType = require('./BridgeType.js');
let Timeout = require('./Timeout.js');
let MapHandle = require('./MapHandle.js');
let NetworkNotification = require('./NetworkNotification.js');
let GripperMode = require('./GripperMode.js');
let LedState = require('./LedState.js');
let WaypointList = require('./WaypointList.js');
let ControllerElementEventType = require('./ControllerElementEventType.js');
let FactoryEvent = require('./FactoryEvent.js');
let WifiSecurityType = require('./WifiSecurityType.js');
let Base_RotationMatrixRow = require('./Base_RotationMatrixRow.js');
let GpioBehavior = require('./GpioBehavior.js');
let ConfigurationChangeNotificationList = require('./ConfigurationChangeNotificationList.js');
let TrajectoryErrorIdentifier = require('./TrajectoryErrorIdentifier.js');
let Gen3GpioPinId = require('./Gen3GpioPinId.js');
let ServoingModeNotificationList = require('./ServoingModeNotificationList.js');
let ControllerElementHandle_identifier = require('./ControllerElementHandle_identifier.js');
let ControllerConfigurationMode = require('./ControllerConfigurationMode.js');
let Base_ControlModeInformation = require('./Base_ControlModeInformation.js');
let RobotEvent = require('./RobotEvent.js');
let Xbox360DigitalInputIdentifier = require('./Xbox360DigitalInputIdentifier.js');
let MapEvent = require('./MapEvent.js');
let CartesianWaypoint = require('./CartesianWaypoint.js');
let ActionEvent = require('./ActionEvent.js');
let SnapshotType = require('./SnapshotType.js');
let UserNotificationList = require('./UserNotificationList.js');
let MapGroupHandle = require('./MapGroupHandle.js');
let SwitchControlMapping = require('./SwitchControlMapping.js');
let MapList = require('./MapList.js');
let ConfigurationChangeNotification = require('./ConfigurationChangeNotification.js');
let CartesianTrajectoryConstraint_type = require('./CartesianTrajectoryConstraint_type.js');
let KinematicTrajectoryConstraints = require('./KinematicTrajectoryConstraints.js');
let MapEvent_events = require('./MapEvent_events.js');
let JointTorques = require('./JointTorques.js');
let AngularWaypoint = require('./AngularWaypoint.js');
let SequenceHandle = require('./SequenceHandle.js');
let ConfigurationChangeNotification_configuration_change = require('./ConfigurationChangeNotification_configuration_change.js');
let MappingInfoNotificationList = require('./MappingInfoNotificationList.js');
let GpioConfigurationList = require('./GpioConfigurationList.js');
let TrajectoryInfo = require('./TrajectoryInfo.js');
let TransformationRow = require('./TransformationRow.js');
let UserProfileList = require('./UserProfileList.js');
let TrajectoryErrorType = require('./TrajectoryErrorType.js');
let GpioPinPropertyFlags = require('./GpioPinPropertyFlags.js');
let JointAngle = require('./JointAngle.js');
let Pose = require('./Pose.js');
let ArmStateNotification = require('./ArmStateNotification.js');
let SequenceTask = require('./SequenceTask.js');
let WristDigitalInputIdentifier = require('./WristDigitalInputIdentifier.js');
let ControllerConfiguration = require('./ControllerConfiguration.js');
let Base_ControlModeNotification = require('./Base_ControlModeNotification.js');
let WifiEncryptionType = require('./WifiEncryptionType.js');
let EventIdSequenceInfoNotification = require('./EventIdSequenceInfoNotification.js');
let TrajectoryErrorElement = require('./TrajectoryErrorElement.js');
let GpioCommand = require('./GpioCommand.js');
let ActuatorInformation = require('./ActuatorInformation.js');
let ChangeWrench = require('./ChangeWrench.js');
let JointSpeed = require('./JointSpeed.js');
let ProtectionZoneNotificationList = require('./ProtectionZoneNotificationList.js');
let LimitationType = require('./LimitationType.js');
let ControllerType = require('./ControllerType.js');
let OperatingMode = require('./OperatingMode.js');
let ConstrainedJointAngle = require('./ConstrainedJointAngle.js');
let ControlModeNotificationList = require('./ControlModeNotificationList.js');
let SafetyEvent = require('./SafetyEvent.js');
let ActionExecutionState = require('./ActionExecutionState.js');
let SequenceInfoNotification = require('./SequenceInfoNotification.js');
let ControllerEventType = require('./ControllerEventType.js');
let ControllerEvent = require('./ControllerEvent.js');
let EmergencyStop = require('./EmergencyStop.js');
let JointTrajectoryConstraintType = require('./JointTrajectoryConstraintType.js');
let ControllerElementState = require('./ControllerElementState.js');
let ProtectionZoneInformation = require('./ProtectionZoneInformation.js');
let Action_action_parameters = require('./Action_action_parameters.js');
let Base_CapSenseMode = require('./Base_CapSenseMode.js');
let RequestedActionType = require('./RequestedActionType.js');
let RobotEventNotificationList = require('./RobotEventNotificationList.js');
let ArmStateInformation = require('./ArmStateInformation.js');
let ActionHandle = require('./ActionHandle.js');
let JointAngles = require('./JointAngles.js');
let MappingList = require('./MappingList.js');
let FactoryNotification = require('./FactoryNotification.js');
let ChangeJointSpeeds = require('./ChangeJointSpeeds.js');
let ProtectionZone = require('./ProtectionZone.js');
let JointLimitation = require('./JointLimitation.js');
let SequenceInfoNotificationList = require('./SequenceInfoNotificationList.js');
let BridgeList = require('./BridgeList.js');
let Orientation = require('./Orientation.js');
let WrenchLimitation = require('./WrenchLimitation.js');
let TrajectoryContinuityMode = require('./TrajectoryContinuityMode.js');
let IKData = require('./IKData.js');
let BaseCyclic_Feedback = require('./BaseCyclic_Feedback.js');
let BaseFeedback = require('./BaseFeedback.js');
let BaseCyclic_Command = require('./BaseCyclic_Command.js');
let ActuatorCommand = require('./ActuatorCommand.js');
let ActuatorFeedback = require('./ActuatorFeedback.js');
let BaseCyclic_CustomData = require('./BaseCyclic_CustomData.js');
let BaseCyclic_ServiceVersion = require('./BaseCyclic_ServiceVersion.js');
let ActuatorCustomData = require('./ActuatorCustomData.js');
let UARTConfiguration = require('./UARTConfiguration.js');
let NotificationType = require('./NotificationType.js');
let ArmState = require('./ArmState.js');
let UARTSpeed = require('./UARTSpeed.js');
let SafetyStatusValue = require('./SafetyStatusValue.js');
let DeviceTypes = require('./DeviceTypes.js');
let Connection = require('./Connection.js');
let CountryCodeIdentifier = require('./CountryCodeIdentifier.js');
let UARTDeviceIdentification = require('./UARTDeviceIdentification.js');
let Permission = require('./Permission.js');
let CartesianReferenceFrame = require('./CartesianReferenceFrame.js');
let UARTParity = require('./UARTParity.js');
let UserProfileHandle = require('./UserProfileHandle.js');
let UARTStopBits = require('./UARTStopBits.js');
let NotificationOptions = require('./NotificationOptions.js');
let Timestamp = require('./Timestamp.js');
let Unit = require('./Unit.js');
let NotificationHandle = require('./NotificationHandle.js');
let DeviceHandle = require('./DeviceHandle.js');
let Empty = require('./Empty.js');
let UARTWordLength = require('./UARTWordLength.js');
let SafetyNotification = require('./SafetyNotification.js');
let SafetyHandle = require('./SafetyHandle.js');
let CountryCode = require('./CountryCode.js');
let AngularTwist = require('./AngularTwist.js');
let ControlConfig_JointSpeeds = require('./ControlConfig_JointSpeeds.js');
let ControlConfig_ControlMode = require('./ControlConfig_ControlMode.js');
let ControlConfigurationEvent = require('./ControlConfigurationEvent.js');
let DesiredSpeeds = require('./DesiredSpeeds.js');
let TwistAngularSoftLimit = require('./TwistAngularSoftLimit.js');
let ControlConfig_ControlModeNotification = require('./ControlConfig_ControlModeNotification.js');
let TwistLinearSoftLimit = require('./TwistLinearSoftLimit.js');
let KinematicLimitsList = require('./KinematicLimitsList.js');
let GravityVector = require('./GravityVector.js');
let CartesianTransform = require('./CartesianTransform.js');
let KinematicLimits = require('./KinematicLimits.js');
let PayloadInformation = require('./PayloadInformation.js');
let JointAccelerationSoftLimits = require('./JointAccelerationSoftLimits.js');
let ControlConfig_ServiceVersion = require('./ControlConfig_ServiceVersion.js');
let CartesianReferenceFrameInfo = require('./CartesianReferenceFrameInfo.js');
let ControlConfig_Position = require('./ControlConfig_Position.js');
let ToolConfiguration = require('./ToolConfiguration.js');
let ControlConfigurationNotification = require('./ControlConfigurationNotification.js');
let LinearTwist = require('./LinearTwist.js');
let JointSpeedSoftLimits = require('./JointSpeedSoftLimits.js');
let ControlConfig_ControlModeInformation = require('./ControlConfig_ControlModeInformation.js');
let SafetyConfiguration = require('./SafetyConfiguration.js');
let IPv4Settings = require('./IPv4Settings.js');
let CalibrationItem = require('./CalibrationItem.js');
let CalibrationParameter_value = require('./CalibrationParameter_value.js');
let Calibration = require('./Calibration.js');
let SafetyInformationList = require('./SafetyInformationList.js');
let PartNumber = require('./PartNumber.js');
let DeviceConfig_SafetyLimitType = require('./DeviceConfig_SafetyLimitType.js');
let CapSenseRegister = require('./CapSenseRegister.js');
let CalibrationElement = require('./CalibrationElement.js');
let DeviceConfig_CapSenseMode = require('./DeviceConfig_CapSenseMode.js');
let RebootRqst = require('./RebootRqst.js');
let CalibrationStatus = require('./CalibrationStatus.js');
let CalibrationParameter = require('./CalibrationParameter.js');
let SafetyEnable = require('./SafetyEnable.js');
let PowerOnSelfTestResult = require('./PowerOnSelfTestResult.js');
let PartNumberRevision = require('./PartNumberRevision.js');
let RunModes = require('./RunModes.js');
let ModelNumber = require('./ModelNumber.js');
let FirmwareVersion = require('./FirmwareVersion.js');
let DeviceConfig_ServiceVersion = require('./DeviceConfig_ServiceVersion.js');
let SafetyThreshold = require('./SafetyThreshold.js');
let DeviceConfig_CapSenseConfig = require('./DeviceConfig_CapSenseConfig.js');
let SafetyConfigurationList = require('./SafetyConfigurationList.js');
let BootloaderVersion = require('./BootloaderVersion.js');
let CalibrationResult = require('./CalibrationResult.js');
let SafetyInformation = require('./SafetyInformation.js');
let SerialNumber = require('./SerialNumber.js');
let RunMode = require('./RunMode.js');
let MACAddress = require('./MACAddress.js');
let DeviceType = require('./DeviceType.js');
let SafetyStatus = require('./SafetyStatus.js');
let DeviceManager_ServiceVersion = require('./DeviceManager_ServiceVersion.js');
let DeviceHandles = require('./DeviceHandles.js');
let GripperConfig_SafetyIdentifier = require('./GripperConfig_SafetyIdentifier.js');
let RobotiqGripperStatusFlags = require('./RobotiqGripperStatusFlags.js');
let GripperCyclic_Command = require('./GripperCyclic_Command.js');
let GripperCyclic_Feedback = require('./GripperCyclic_Feedback.js');
let MotorCommand = require('./MotorCommand.js');
let GripperCyclic_CustomData = require('./GripperCyclic_CustomData.js');
let CustomDataUnit = require('./CustomDataUnit.js');
let MotorFeedback = require('./MotorFeedback.js');
let GripperCyclic_ServiceVersion = require('./GripperCyclic_ServiceVersion.js');
let GripperCyclic_MessageId = require('./GripperCyclic_MessageId.js');
let GPIOMode = require('./GPIOMode.js');
let GPIOValue = require('./GPIOValue.js');
let I2CRegisterAddressSize = require('./I2CRegisterAddressSize.js');
let EthernetConfiguration = require('./EthernetConfiguration.js');
let I2CReadParameter = require('./I2CReadParameter.js');
let I2CWriteParameter = require('./I2CWriteParameter.js');
let EthernetDuplex = require('./EthernetDuplex.js');
let I2CDeviceAddressing = require('./I2CDeviceAddressing.js');
let I2CWriteRegisterParameter = require('./I2CWriteRegisterParameter.js');
let I2CDevice = require('./I2CDevice.js');
let UARTPortId = require('./UARTPortId.js');
let I2CConfiguration = require('./I2CConfiguration.js');
let GPIOPull = require('./GPIOPull.js');
let I2CMode = require('./I2CMode.js');
let GPIOState = require('./GPIOState.js');
let InterconnectConfig_SafetyIdentifier = require('./InterconnectConfig_SafetyIdentifier.js');
let EthernetDevice = require('./EthernetDevice.js');
let GPIOIdentification = require('./GPIOIdentification.js');
let EthernetSpeed = require('./EthernetSpeed.js');
let InterconnectConfig_ServiceVersion = require('./InterconnectConfig_ServiceVersion.js');
let EthernetDeviceIdentification = require('./EthernetDeviceIdentification.js');
let GPIOIdentifier = require('./GPIOIdentifier.js');
let I2CReadRegisterParameter = require('./I2CReadRegisterParameter.js');
let I2CData = require('./I2CData.js');
let I2CDeviceIdentification = require('./I2CDeviceIdentification.js');
let InterconnectConfig_GPIOConfiguration = require('./InterconnectConfig_GPIOConfiguration.js');
let InterconnectCyclic_Feedback = require('./InterconnectCyclic_Feedback.js');
let InterconnectCyclic_Command = require('./InterconnectCyclic_Command.js');
let InterconnectCyclic_CustomData = require('./InterconnectCyclic_CustomData.js');
let InterconnectCyclic_Feedback_tool_feedback = require('./InterconnectCyclic_Feedback_tool_feedback.js');
let InterconnectCyclic_MessageId = require('./InterconnectCyclic_MessageId.js');
let InterconnectCyclic_ServiceVersion = require('./InterconnectCyclic_ServiceVersion.js');
let InterconnectCyclic_Command_tool_command = require('./InterconnectCyclic_Command_tool_command.js');
let InterconnectCyclic_CustomData_tool_customData = require('./InterconnectCyclic_CustomData_tool_customData.js');
let VisionModuleType = require('./VisionModuleType.js');
let EndEffectorType = require('./EndEffectorType.js');
let WristType = require('./WristType.js');
let BrakeType = require('./BrakeType.js');
let BaseType = require('./BaseType.js');
let ArmLaterality = require('./ArmLaterality.js');
let ModelId = require('./ModelId.js');
let ProductConfigurationEndEffectorType = require('./ProductConfigurationEndEffectorType.js');
let InterfaceModuleType = require('./InterfaceModuleType.js');
let CompleteProductConfiguration = require('./CompleteProductConfiguration.js');
let VisionConfig_RotationMatrixRow = require('./VisionConfig_RotationMatrixRow.js');
let VisionEvent = require('./VisionEvent.js');
let ManualFocus = require('./ManualFocus.js');
let SensorFocusAction_action_parameters = require('./SensorFocusAction_action_parameters.js');
let Sensor = require('./Sensor.js');
let VisionNotification = require('./VisionNotification.js');
let IntrinsicParameters = require('./IntrinsicParameters.js');
let Resolution = require('./Resolution.js');
let OptionValue = require('./OptionValue.js');
let OptionInformation = require('./OptionInformation.js');
let IntrinsicProfileIdentifier = require('./IntrinsicProfileIdentifier.js');
let FocusPoint = require('./FocusPoint.js');
let Option = require('./Option.js');
let FrameRate = require('./FrameRate.js');
let SensorFocusAction = require('./SensorFocusAction.js');
let TranslationVector = require('./TranslationVector.js');
let FocusAction = require('./FocusAction.js');
let BitRate = require('./BitRate.js');
let SensorIdentifier = require('./SensorIdentifier.js');
let VisionConfig_RotationMatrix = require('./VisionConfig_RotationMatrix.js');
let SensorSettings = require('./SensorSettings.js');
let OptionIdentifier = require('./OptionIdentifier.js');
let ExtrinsicParameters = require('./ExtrinsicParameters.js');
let DistortionCoefficients = require('./DistortionCoefficients.js');
let VisionConfig_ServiceVersion = require('./VisionConfig_ServiceVersion.js');
let FollowCartesianTrajectoryActionResult = require('./FollowCartesianTrajectoryActionResult.js');
let FollowCartesianTrajectoryGoal = require('./FollowCartesianTrajectoryGoal.js');
let FollowCartesianTrajectoryAction = require('./FollowCartesianTrajectoryAction.js');
let FollowCartesianTrajectoryFeedback = require('./FollowCartesianTrajectoryFeedback.js');
let FollowCartesianTrajectoryActionGoal = require('./FollowCartesianTrajectoryActionGoal.js');
let FollowCartesianTrajectoryResult = require('./FollowCartesianTrajectoryResult.js');
let FollowCartesianTrajectoryActionFeedback = require('./FollowCartesianTrajectoryActionFeedback.js');

module.exports = {
  ApiOptions: ApiOptions,
  KortexError: KortexError,
  SubErrorCodes: SubErrorCodes,
  ErrorCodes: ErrorCodes,
  FrequencyResponse: FrequencyResponse,
  ControlLoop: ControlLoop,
  StepResponse: StepResponse,
  CommandMode: CommandMode,
  ActuatorConfig_SafetyLimitType: ActuatorConfig_SafetyLimitType,
  ControlLoopParameters: ControlLoopParameters,
  ActuatorConfig_ControlMode: ActuatorConfig_ControlMode,
  LoopSelection: LoopSelection,
  CustomDataIndex: CustomDataIndex,
  CommandModeInformation: CommandModeInformation,
  RampResponse: RampResponse,
  SafetyIdentifierBankA: SafetyIdentifierBankA,
  TorqueOffset: TorqueOffset,
  CoggingFeedforwardMode: CoggingFeedforwardMode,
  ControlLoopSelection: ControlLoopSelection,
  AxisPosition: AxisPosition,
  VectorDriveParameters: VectorDriveParameters,
  AxisOffsets: AxisOffsets,
  TorqueCalibration: TorqueCalibration,
  EncoderDerivativeParameters: EncoderDerivativeParameters,
  CoggingFeedforwardModeInformation: CoggingFeedforwardModeInformation,
  ActuatorConfig_ControlModeInformation: ActuatorConfig_ControlModeInformation,
  PositionCommand: PositionCommand,
  ActuatorConfig_ServiceVersion: ActuatorConfig_ServiceVersion,
  Servoing: Servoing,
  CustomDataSelection: CustomDataSelection,
  ActuatorCyclic_MessageId: ActuatorCyclic_MessageId,
  StatusFlags: StatusFlags,
  ActuatorCyclic_CustomData: ActuatorCyclic_CustomData,
  CommandFlags: CommandFlags,
  ActuatorCyclic_Command: ActuatorCyclic_Command,
  ActuatorCyclic_ServiceVersion: ActuatorCyclic_ServiceVersion,
  ActuatorCyclic_Feedback: ActuatorCyclic_Feedback,
  WifiInformation: WifiInformation,
  BridgeConfig: BridgeConfig,
  PasswordChange: PasswordChange,
  BridgeStatus: BridgeStatus,
  PreComputedJointTrajectory: PreComputedJointTrajectory,
  SystemTime: SystemTime,
  BridgePortConfig: BridgePortConfig,
  ActionNotification: ActionNotification,
  UserNotification: UserNotification,
  GripperCommand: GripperCommand,
  Delay: Delay,
  WifiConfigurationList: WifiConfigurationList,
  SequenceTaskHandle: SequenceTaskHandle,
  CartesianTrajectoryConstraint: CartesianTrajectoryConstraint,
  ServoingModeNotification: ServoingModeNotification,
  Base_RotationMatrix: Base_RotationMatrix,
  TransformationMatrix: TransformationMatrix,
  NetworkEvent: NetworkEvent,
  SequenceTasks: SequenceTasks,
  ConstrainedJointAngles: ConstrainedJointAngles,
  Gripper: Gripper,
  SignalQuality: SignalQuality,
  SequenceTasksPair: SequenceTasksPair,
  ServoingMode: ServoingMode,
  IPv4Information: IPv4Information,
  ActivateMapHandle: ActivateMapHandle,
  Wrench: Wrench,
  Admittance: Admittance,
  WaypointValidationReport: WaypointValidationReport,
  UserList: UserList,
  ControllerInputType: ControllerInputType,
  GripperRequest: GripperRequest,
  MapGroupList: MapGroupList,
  Base_SafetyIdentifier: Base_SafetyIdentifier,
  SafetyNotificationList: SafetyNotificationList,
  JointsLimitationsList: JointsLimitationsList,
  CartesianLimitation: CartesianLimitation,
  MappingHandle: MappingHandle,
  MapGroup: MapGroup,
  Base_ControlMode: Base_ControlMode,
  WifiEnableState: WifiEnableState,
  SequenceTasksConfiguration: SequenceTasksConfiguration,
  Mapping: Mapping,
  Base_Position: Base_Position,
  ControllerNotification: ControllerNotification,
  ActionList: ActionList,
  Ssid: Ssid,
  WifiInformationList: WifiInformationList,
  JointTorque: JointTorque,
  FirmwareBundleVersions: FirmwareBundleVersions,
  Finger: Finger,
  BridgeResult: BridgeResult,
  WrenchMode: WrenchMode,
  ControllerBehavior: ControllerBehavior,
  NetworkNotificationList: NetworkNotificationList,
  TrajectoryErrorReport: TrajectoryErrorReport,
  Sequence: Sequence,
  ShapeType: ShapeType,
  ControllerHandle: ControllerHandle,
  CommunicationInterfaceConfiguration: CommunicationInterfaceConfiguration,
  MappingInfoNotification: MappingInfoNotification,
  TrajectoryInfoType: TrajectoryInfoType,
  ControllerList: ControllerList,
  MapElement: MapElement,
  Action: Action,
  Faults: Faults,
  UserEvent: UserEvent,
  AdvancedSequenceHandle: AdvancedSequenceHandle,
  ConstrainedPose: ConstrainedPose,
  FullIPv4Configuration: FullIPv4Configuration,
  FullUserProfile: FullUserProfile,
  CartesianSpeed: CartesianSpeed,
  ConstrainedOrientation: ConstrainedOrientation,
  TwistLimitation: TwistLimitation,
  ActionType: ActionType,
  ConstrainedPosition: ConstrainedPosition,
  OperatingModeNotificationList: OperatingModeNotificationList,
  Base_ServiceVersion: Base_ServiceVersion,
  BridgeIdentifier: BridgeIdentifier,
  Base_GpioConfiguration: Base_GpioConfiguration,
  JointTrajectoryConstraint: JointTrajectoryConstraint,
  ActionNotificationList: ActionNotificationList,
  SequenceTasksRange: SequenceTasksRange,
  NavigationDirection: NavigationDirection,
  RobotEventNotification: RobotEventNotification,
  NetworkType: NetworkType,
  OperatingModeInformation: OperatingModeInformation,
  Base_CapSenseConfig: Base_CapSenseConfig,
  GpioPinConfiguration: GpioPinConfiguration,
  SequenceList: SequenceList,
  BluetoothEnableState: BluetoothEnableState,
  OperatingModeNotification: OperatingModeNotification,
  Xbox360AnalogInputIdentifier: Xbox360AnalogInputIdentifier,
  Waypoint: Waypoint,
  ZoneShape: ZoneShape,
  IPv4Configuration: IPv4Configuration,
  TwistCommand: TwistCommand,
  ProtectionZoneList: ProtectionZoneList,
  WifiConfiguration: WifiConfiguration,
  FirmwareComponentVersion: FirmwareComponentVersion,
  ControllerElementHandle: ControllerElementHandle,
  Waypoint_type_of_waypoint: Waypoint_type_of_waypoint,
  UserProfile: UserProfile,
  SequenceTaskConfiguration: SequenceTaskConfiguration,
  Map: Map,
  Point: Point,
  ProtectionZoneEvent: ProtectionZoneEvent,
  ConfigurationNotificationEvent: ConfigurationNotificationEvent,
  Snapshot: Snapshot,
  WrenchCommand: WrenchCommand,
  JointNavigationDirection: JointNavigationDirection,
  Base_Stop: Base_Stop,
  ControllerConfigurationList: ControllerConfigurationList,
  ControllerNotification_state: ControllerNotification_state,
  ServoingModeInformation: ServoingModeInformation,
  ControllerState: ControllerState,
  NetworkHandle: NetworkHandle,
  Base_JointSpeeds: Base_JointSpeeds,
  ProtectionZoneNotification: ProtectionZoneNotification,
  GpioEvent: GpioEvent,
  GpioAction: GpioAction,
  SoundType: SoundType,
  SequenceInformation: SequenceInformation,
  PreComputedJointTrajectoryElement: PreComputedJointTrajectoryElement,
  ProtectionZoneHandle: ProtectionZoneHandle,
  BackupEvent: BackupEvent,
  Query: Query,
  AppendActionInformation: AppendActionInformation,
  ChangeTwist: ChangeTwist,
  Twist: Twist,
  ControllerNotificationList: ControllerNotificationList,
  AdmittanceMode: AdmittanceMode,
  CartesianLimitationList: CartesianLimitationList,
  RFConfiguration: RFConfiguration,
  BridgeType: BridgeType,
  Timeout: Timeout,
  MapHandle: MapHandle,
  NetworkNotification: NetworkNotification,
  GripperMode: GripperMode,
  LedState: LedState,
  WaypointList: WaypointList,
  ControllerElementEventType: ControllerElementEventType,
  FactoryEvent: FactoryEvent,
  WifiSecurityType: WifiSecurityType,
  Base_RotationMatrixRow: Base_RotationMatrixRow,
  GpioBehavior: GpioBehavior,
  ConfigurationChangeNotificationList: ConfigurationChangeNotificationList,
  TrajectoryErrorIdentifier: TrajectoryErrorIdentifier,
  Gen3GpioPinId: Gen3GpioPinId,
  ServoingModeNotificationList: ServoingModeNotificationList,
  ControllerElementHandle_identifier: ControllerElementHandle_identifier,
  ControllerConfigurationMode: ControllerConfigurationMode,
  Base_ControlModeInformation: Base_ControlModeInformation,
  RobotEvent: RobotEvent,
  Xbox360DigitalInputIdentifier: Xbox360DigitalInputIdentifier,
  MapEvent: MapEvent,
  CartesianWaypoint: CartesianWaypoint,
  ActionEvent: ActionEvent,
  SnapshotType: SnapshotType,
  UserNotificationList: UserNotificationList,
  MapGroupHandle: MapGroupHandle,
  SwitchControlMapping: SwitchControlMapping,
  MapList: MapList,
  ConfigurationChangeNotification: ConfigurationChangeNotification,
  CartesianTrajectoryConstraint_type: CartesianTrajectoryConstraint_type,
  KinematicTrajectoryConstraints: KinematicTrajectoryConstraints,
  MapEvent_events: MapEvent_events,
  JointTorques: JointTorques,
  AngularWaypoint: AngularWaypoint,
  SequenceHandle: SequenceHandle,
  ConfigurationChangeNotification_configuration_change: ConfigurationChangeNotification_configuration_change,
  MappingInfoNotificationList: MappingInfoNotificationList,
  GpioConfigurationList: GpioConfigurationList,
  TrajectoryInfo: TrajectoryInfo,
  TransformationRow: TransformationRow,
  UserProfileList: UserProfileList,
  TrajectoryErrorType: TrajectoryErrorType,
  GpioPinPropertyFlags: GpioPinPropertyFlags,
  JointAngle: JointAngle,
  Pose: Pose,
  ArmStateNotification: ArmStateNotification,
  SequenceTask: SequenceTask,
  WristDigitalInputIdentifier: WristDigitalInputIdentifier,
  ControllerConfiguration: ControllerConfiguration,
  Base_ControlModeNotification: Base_ControlModeNotification,
  WifiEncryptionType: WifiEncryptionType,
  EventIdSequenceInfoNotification: EventIdSequenceInfoNotification,
  TrajectoryErrorElement: TrajectoryErrorElement,
  GpioCommand: GpioCommand,
  ActuatorInformation: ActuatorInformation,
  ChangeWrench: ChangeWrench,
  JointSpeed: JointSpeed,
  ProtectionZoneNotificationList: ProtectionZoneNotificationList,
  LimitationType: LimitationType,
  ControllerType: ControllerType,
  OperatingMode: OperatingMode,
  ConstrainedJointAngle: ConstrainedJointAngle,
  ControlModeNotificationList: ControlModeNotificationList,
  SafetyEvent: SafetyEvent,
  ActionExecutionState: ActionExecutionState,
  SequenceInfoNotification: SequenceInfoNotification,
  ControllerEventType: ControllerEventType,
  ControllerEvent: ControllerEvent,
  EmergencyStop: EmergencyStop,
  JointTrajectoryConstraintType: JointTrajectoryConstraintType,
  ControllerElementState: ControllerElementState,
  ProtectionZoneInformation: ProtectionZoneInformation,
  Action_action_parameters: Action_action_parameters,
  Base_CapSenseMode: Base_CapSenseMode,
  RequestedActionType: RequestedActionType,
  RobotEventNotificationList: RobotEventNotificationList,
  ArmStateInformation: ArmStateInformation,
  ActionHandle: ActionHandle,
  JointAngles: JointAngles,
  MappingList: MappingList,
  FactoryNotification: FactoryNotification,
  ChangeJointSpeeds: ChangeJointSpeeds,
  ProtectionZone: ProtectionZone,
  JointLimitation: JointLimitation,
  SequenceInfoNotificationList: SequenceInfoNotificationList,
  BridgeList: BridgeList,
  Orientation: Orientation,
  WrenchLimitation: WrenchLimitation,
  TrajectoryContinuityMode: TrajectoryContinuityMode,
  IKData: IKData,
  BaseCyclic_Feedback: BaseCyclic_Feedback,
  BaseFeedback: BaseFeedback,
  BaseCyclic_Command: BaseCyclic_Command,
  ActuatorCommand: ActuatorCommand,
  ActuatorFeedback: ActuatorFeedback,
  BaseCyclic_CustomData: BaseCyclic_CustomData,
  BaseCyclic_ServiceVersion: BaseCyclic_ServiceVersion,
  ActuatorCustomData: ActuatorCustomData,
  UARTConfiguration: UARTConfiguration,
  NotificationType: NotificationType,
  ArmState: ArmState,
  UARTSpeed: UARTSpeed,
  SafetyStatusValue: SafetyStatusValue,
  DeviceTypes: DeviceTypes,
  Connection: Connection,
  CountryCodeIdentifier: CountryCodeIdentifier,
  UARTDeviceIdentification: UARTDeviceIdentification,
  Permission: Permission,
  CartesianReferenceFrame: CartesianReferenceFrame,
  UARTParity: UARTParity,
  UserProfileHandle: UserProfileHandle,
  UARTStopBits: UARTStopBits,
  NotificationOptions: NotificationOptions,
  Timestamp: Timestamp,
  Unit: Unit,
  NotificationHandle: NotificationHandle,
  DeviceHandle: DeviceHandle,
  Empty: Empty,
  UARTWordLength: UARTWordLength,
  SafetyNotification: SafetyNotification,
  SafetyHandle: SafetyHandle,
  CountryCode: CountryCode,
  AngularTwist: AngularTwist,
  ControlConfig_JointSpeeds: ControlConfig_JointSpeeds,
  ControlConfig_ControlMode: ControlConfig_ControlMode,
  ControlConfigurationEvent: ControlConfigurationEvent,
  DesiredSpeeds: DesiredSpeeds,
  TwistAngularSoftLimit: TwistAngularSoftLimit,
  ControlConfig_ControlModeNotification: ControlConfig_ControlModeNotification,
  TwistLinearSoftLimit: TwistLinearSoftLimit,
  KinematicLimitsList: KinematicLimitsList,
  GravityVector: GravityVector,
  CartesianTransform: CartesianTransform,
  KinematicLimits: KinematicLimits,
  PayloadInformation: PayloadInformation,
  JointAccelerationSoftLimits: JointAccelerationSoftLimits,
  ControlConfig_ServiceVersion: ControlConfig_ServiceVersion,
  CartesianReferenceFrameInfo: CartesianReferenceFrameInfo,
  ControlConfig_Position: ControlConfig_Position,
  ToolConfiguration: ToolConfiguration,
  ControlConfigurationNotification: ControlConfigurationNotification,
  LinearTwist: LinearTwist,
  JointSpeedSoftLimits: JointSpeedSoftLimits,
  ControlConfig_ControlModeInformation: ControlConfig_ControlModeInformation,
  SafetyConfiguration: SafetyConfiguration,
  IPv4Settings: IPv4Settings,
  CalibrationItem: CalibrationItem,
  CalibrationParameter_value: CalibrationParameter_value,
  Calibration: Calibration,
  SafetyInformationList: SafetyInformationList,
  PartNumber: PartNumber,
  DeviceConfig_SafetyLimitType: DeviceConfig_SafetyLimitType,
  CapSenseRegister: CapSenseRegister,
  CalibrationElement: CalibrationElement,
  DeviceConfig_CapSenseMode: DeviceConfig_CapSenseMode,
  RebootRqst: RebootRqst,
  CalibrationStatus: CalibrationStatus,
  CalibrationParameter: CalibrationParameter,
  SafetyEnable: SafetyEnable,
  PowerOnSelfTestResult: PowerOnSelfTestResult,
  PartNumberRevision: PartNumberRevision,
  RunModes: RunModes,
  ModelNumber: ModelNumber,
  FirmwareVersion: FirmwareVersion,
  DeviceConfig_ServiceVersion: DeviceConfig_ServiceVersion,
  SafetyThreshold: SafetyThreshold,
  DeviceConfig_CapSenseConfig: DeviceConfig_CapSenseConfig,
  SafetyConfigurationList: SafetyConfigurationList,
  BootloaderVersion: BootloaderVersion,
  CalibrationResult: CalibrationResult,
  SafetyInformation: SafetyInformation,
  SerialNumber: SerialNumber,
  RunMode: RunMode,
  MACAddress: MACAddress,
  DeviceType: DeviceType,
  SafetyStatus: SafetyStatus,
  DeviceManager_ServiceVersion: DeviceManager_ServiceVersion,
  DeviceHandles: DeviceHandles,
  GripperConfig_SafetyIdentifier: GripperConfig_SafetyIdentifier,
  RobotiqGripperStatusFlags: RobotiqGripperStatusFlags,
  GripperCyclic_Command: GripperCyclic_Command,
  GripperCyclic_Feedback: GripperCyclic_Feedback,
  MotorCommand: MotorCommand,
  GripperCyclic_CustomData: GripperCyclic_CustomData,
  CustomDataUnit: CustomDataUnit,
  MotorFeedback: MotorFeedback,
  GripperCyclic_ServiceVersion: GripperCyclic_ServiceVersion,
  GripperCyclic_MessageId: GripperCyclic_MessageId,
  GPIOMode: GPIOMode,
  GPIOValue: GPIOValue,
  I2CRegisterAddressSize: I2CRegisterAddressSize,
  EthernetConfiguration: EthernetConfiguration,
  I2CReadParameter: I2CReadParameter,
  I2CWriteParameter: I2CWriteParameter,
  EthernetDuplex: EthernetDuplex,
  I2CDeviceAddressing: I2CDeviceAddressing,
  I2CWriteRegisterParameter: I2CWriteRegisterParameter,
  I2CDevice: I2CDevice,
  UARTPortId: UARTPortId,
  I2CConfiguration: I2CConfiguration,
  GPIOPull: GPIOPull,
  I2CMode: I2CMode,
  GPIOState: GPIOState,
  InterconnectConfig_SafetyIdentifier: InterconnectConfig_SafetyIdentifier,
  EthernetDevice: EthernetDevice,
  GPIOIdentification: GPIOIdentification,
  EthernetSpeed: EthernetSpeed,
  InterconnectConfig_ServiceVersion: InterconnectConfig_ServiceVersion,
  EthernetDeviceIdentification: EthernetDeviceIdentification,
  GPIOIdentifier: GPIOIdentifier,
  I2CReadRegisterParameter: I2CReadRegisterParameter,
  I2CData: I2CData,
  I2CDeviceIdentification: I2CDeviceIdentification,
  InterconnectConfig_GPIOConfiguration: InterconnectConfig_GPIOConfiguration,
  InterconnectCyclic_Feedback: InterconnectCyclic_Feedback,
  InterconnectCyclic_Command: InterconnectCyclic_Command,
  InterconnectCyclic_CustomData: InterconnectCyclic_CustomData,
  InterconnectCyclic_Feedback_tool_feedback: InterconnectCyclic_Feedback_tool_feedback,
  InterconnectCyclic_MessageId: InterconnectCyclic_MessageId,
  InterconnectCyclic_ServiceVersion: InterconnectCyclic_ServiceVersion,
  InterconnectCyclic_Command_tool_command: InterconnectCyclic_Command_tool_command,
  InterconnectCyclic_CustomData_tool_customData: InterconnectCyclic_CustomData_tool_customData,
  VisionModuleType: VisionModuleType,
  EndEffectorType: EndEffectorType,
  WristType: WristType,
  BrakeType: BrakeType,
  BaseType: BaseType,
  ArmLaterality: ArmLaterality,
  ModelId: ModelId,
  ProductConfigurationEndEffectorType: ProductConfigurationEndEffectorType,
  InterfaceModuleType: InterfaceModuleType,
  CompleteProductConfiguration: CompleteProductConfiguration,
  VisionConfig_RotationMatrixRow: VisionConfig_RotationMatrixRow,
  VisionEvent: VisionEvent,
  ManualFocus: ManualFocus,
  SensorFocusAction_action_parameters: SensorFocusAction_action_parameters,
  Sensor: Sensor,
  VisionNotification: VisionNotification,
  IntrinsicParameters: IntrinsicParameters,
  Resolution: Resolution,
  OptionValue: OptionValue,
  OptionInformation: OptionInformation,
  IntrinsicProfileIdentifier: IntrinsicProfileIdentifier,
  FocusPoint: FocusPoint,
  Option: Option,
  FrameRate: FrameRate,
  SensorFocusAction: SensorFocusAction,
  TranslationVector: TranslationVector,
  FocusAction: FocusAction,
  BitRate: BitRate,
  SensorIdentifier: SensorIdentifier,
  VisionConfig_RotationMatrix: VisionConfig_RotationMatrix,
  SensorSettings: SensorSettings,
  OptionIdentifier: OptionIdentifier,
  ExtrinsicParameters: ExtrinsicParameters,
  DistortionCoefficients: DistortionCoefficients,
  VisionConfig_ServiceVersion: VisionConfig_ServiceVersion,
  FollowCartesianTrajectoryActionResult: FollowCartesianTrajectoryActionResult,
  FollowCartesianTrajectoryGoal: FollowCartesianTrajectoryGoal,
  FollowCartesianTrajectoryAction: FollowCartesianTrajectoryAction,
  FollowCartesianTrajectoryFeedback: FollowCartesianTrajectoryFeedback,
  FollowCartesianTrajectoryActionGoal: FollowCartesianTrajectoryActionGoal,
  FollowCartesianTrajectoryResult: FollowCartesianTrajectoryResult,
  FollowCartesianTrajectoryActionFeedback: FollowCartesianTrajectoryActionFeedback,
};

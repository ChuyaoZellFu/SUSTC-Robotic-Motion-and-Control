; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetTrajectoryErrorReport-request.msg.html

(cl:defclass <GetTrajectoryErrorReport-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetTrajectoryErrorReport-request (<GetTrajectoryErrorReport-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetTrajectoryErrorReport-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetTrajectoryErrorReport-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetTrajectoryErrorReport-request> is deprecated: use kortex_driver-srv:GetTrajectoryErrorReport-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetTrajectoryErrorReport-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetTrajectoryErrorReport-request>) ostream)
  "Serializes a message object of type '<GetTrajectoryErrorReport-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetTrajectoryErrorReport-request>) istream)
  "Deserializes a message object of type '<GetTrajectoryErrorReport-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetTrajectoryErrorReport-request>)))
  "Returns string type for a service object of type '<GetTrajectoryErrorReport-request>"
  "kortex_driver/GetTrajectoryErrorReportRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTrajectoryErrorReport-request)))
  "Returns string type for a service object of type 'GetTrajectoryErrorReport-request"
  "kortex_driver/GetTrajectoryErrorReportRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetTrajectoryErrorReport-request>)))
  "Returns md5sum for a message object of type '<GetTrajectoryErrorReport-request>"
  "256dc258079fa086775c57ef6b093f4a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetTrajectoryErrorReport-request)))
  "Returns md5sum for a message object of type 'GetTrajectoryErrorReport-request"
  "256dc258079fa086775c57ef6b093f4a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetTrajectoryErrorReport-request>)))
  "Returns full string definition for message of type '<GetTrajectoryErrorReport-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetTrajectoryErrorReport-request)))
  "Returns full string definition for message of type 'GetTrajectoryErrorReport-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetTrajectoryErrorReport-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetTrajectoryErrorReport-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetTrajectoryErrorReport-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetTrajectoryErrorReport-response.msg.html

(cl:defclass <GetTrajectoryErrorReport-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:TrajectoryErrorReport
    :initform (cl:make-instance 'kortex_driver-msg:TrajectoryErrorReport)))
)

(cl:defclass GetTrajectoryErrorReport-response (<GetTrajectoryErrorReport-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetTrajectoryErrorReport-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetTrajectoryErrorReport-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetTrajectoryErrorReport-response> is deprecated: use kortex_driver-srv:GetTrajectoryErrorReport-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetTrajectoryErrorReport-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetTrajectoryErrorReport-response>) ostream)
  "Serializes a message object of type '<GetTrajectoryErrorReport-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetTrajectoryErrorReport-response>) istream)
  "Deserializes a message object of type '<GetTrajectoryErrorReport-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetTrajectoryErrorReport-response>)))
  "Returns string type for a service object of type '<GetTrajectoryErrorReport-response>"
  "kortex_driver/GetTrajectoryErrorReportResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTrajectoryErrorReport-response)))
  "Returns string type for a service object of type 'GetTrajectoryErrorReport-response"
  "kortex_driver/GetTrajectoryErrorReportResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetTrajectoryErrorReport-response>)))
  "Returns md5sum for a message object of type '<GetTrajectoryErrorReport-response>"
  "256dc258079fa086775c57ef6b093f4a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetTrajectoryErrorReport-response)))
  "Returns md5sum for a message object of type 'GetTrajectoryErrorReport-response"
  "256dc258079fa086775c57ef6b093f4a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetTrajectoryErrorReport-response>)))
  "Returns full string definition for message of type '<GetTrajectoryErrorReport-response>"
  (cl:format cl:nil "TrajectoryErrorReport output~%~%================================================================================~%MSG: kortex_driver/TrajectoryErrorReport~%~%TrajectoryErrorElement[] trajectory_error_elements~%================================================================================~%MSG: kortex_driver/TrajectoryErrorElement~%~%uint32 error_type~%uint32 error_identifier~%float32 error_value~%float32 min_value~%float32 max_value~%uint32 index~%string message~%uint32 waypoint_index~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetTrajectoryErrorReport-response)))
  "Returns full string definition for message of type 'GetTrajectoryErrorReport-response"
  (cl:format cl:nil "TrajectoryErrorReport output~%~%================================================================================~%MSG: kortex_driver/TrajectoryErrorReport~%~%TrajectoryErrorElement[] trajectory_error_elements~%================================================================================~%MSG: kortex_driver/TrajectoryErrorElement~%~%uint32 error_type~%uint32 error_identifier~%float32 error_value~%float32 min_value~%float32 max_value~%uint32 index~%string message~%uint32 waypoint_index~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetTrajectoryErrorReport-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetTrajectoryErrorReport-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetTrajectoryErrorReport-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetTrajectoryErrorReport)))
  'GetTrajectoryErrorReport-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetTrajectoryErrorReport)))
  'GetTrajectoryErrorReport-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTrajectoryErrorReport)))
  "Returns string type for a service object of type '<GetTrajectoryErrorReport>"
  "kortex_driver/GetTrajectoryErrorReport")
; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude PlayCartesianTrajectoryPosition-request.msg.html

(cl:defclass <PlayCartesianTrajectoryPosition-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:ConstrainedPosition
    :initform (cl:make-instance 'kortex_driver-msg:ConstrainedPosition)))
)

(cl:defclass PlayCartesianTrajectoryPosition-request (<PlayCartesianTrajectoryPosition-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlayCartesianTrajectoryPosition-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlayCartesianTrajectoryPosition-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<PlayCartesianTrajectoryPosition-request> is deprecated: use kortex_driver-srv:PlayCartesianTrajectoryPosition-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <PlayCartesianTrajectoryPosition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlayCartesianTrajectoryPosition-request>) ostream)
  "Serializes a message object of type '<PlayCartesianTrajectoryPosition-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlayCartesianTrajectoryPosition-request>) istream)
  "Deserializes a message object of type '<PlayCartesianTrajectoryPosition-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlayCartesianTrajectoryPosition-request>)))
  "Returns string type for a service object of type '<PlayCartesianTrajectoryPosition-request>"
  "kortex_driver/PlayCartesianTrajectoryPositionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlayCartesianTrajectoryPosition-request)))
  "Returns string type for a service object of type 'PlayCartesianTrajectoryPosition-request"
  "kortex_driver/PlayCartesianTrajectoryPositionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlayCartesianTrajectoryPosition-request>)))
  "Returns md5sum for a message object of type '<PlayCartesianTrajectoryPosition-request>"
  "da1eaa51dfbdd2438263c790b42891de")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlayCartesianTrajectoryPosition-request)))
  "Returns md5sum for a message object of type 'PlayCartesianTrajectoryPosition-request"
  "da1eaa51dfbdd2438263c790b42891de")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlayCartesianTrajectoryPosition-request>)))
  "Returns full string definition for message of type '<PlayCartesianTrajectoryPosition-request>"
  (cl:format cl:nil "ConstrainedPosition input~%~%================================================================================~%MSG: kortex_driver/ConstrainedPosition~%~%Base_Position target_position~%CartesianTrajectoryConstraint constraint~%================================================================================~%MSG: kortex_driver/Base_Position~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: kortex_driver/CartesianTrajectoryConstraint~%~%CartesianTrajectoryConstraint_type oneof_type~%================================================================================~%MSG: kortex_driver/CartesianTrajectoryConstraint_type~%~%CartesianSpeed[] speed~%uint32[] duration~%================================================================================~%MSG: kortex_driver/CartesianSpeed~%~%float32 translation~%float32 orientation~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlayCartesianTrajectoryPosition-request)))
  "Returns full string definition for message of type 'PlayCartesianTrajectoryPosition-request"
  (cl:format cl:nil "ConstrainedPosition input~%~%================================================================================~%MSG: kortex_driver/ConstrainedPosition~%~%Base_Position target_position~%CartesianTrajectoryConstraint constraint~%================================================================================~%MSG: kortex_driver/Base_Position~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: kortex_driver/CartesianTrajectoryConstraint~%~%CartesianTrajectoryConstraint_type oneof_type~%================================================================================~%MSG: kortex_driver/CartesianTrajectoryConstraint_type~%~%CartesianSpeed[] speed~%uint32[] duration~%================================================================================~%MSG: kortex_driver/CartesianSpeed~%~%float32 translation~%float32 orientation~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlayCartesianTrajectoryPosition-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlayCartesianTrajectoryPosition-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PlayCartesianTrajectoryPosition-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude PlayCartesianTrajectoryPosition-response.msg.html

(cl:defclass <PlayCartesianTrajectoryPosition-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass PlayCartesianTrajectoryPosition-response (<PlayCartesianTrajectoryPosition-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlayCartesianTrajectoryPosition-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlayCartesianTrajectoryPosition-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<PlayCartesianTrajectoryPosition-response> is deprecated: use kortex_driver-srv:PlayCartesianTrajectoryPosition-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <PlayCartesianTrajectoryPosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlayCartesianTrajectoryPosition-response>) ostream)
  "Serializes a message object of type '<PlayCartesianTrajectoryPosition-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlayCartesianTrajectoryPosition-response>) istream)
  "Deserializes a message object of type '<PlayCartesianTrajectoryPosition-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlayCartesianTrajectoryPosition-response>)))
  "Returns string type for a service object of type '<PlayCartesianTrajectoryPosition-response>"
  "kortex_driver/PlayCartesianTrajectoryPositionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlayCartesianTrajectoryPosition-response)))
  "Returns string type for a service object of type 'PlayCartesianTrajectoryPosition-response"
  "kortex_driver/PlayCartesianTrajectoryPositionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlayCartesianTrajectoryPosition-response>)))
  "Returns md5sum for a message object of type '<PlayCartesianTrajectoryPosition-response>"
  "da1eaa51dfbdd2438263c790b42891de")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlayCartesianTrajectoryPosition-response)))
  "Returns md5sum for a message object of type 'PlayCartesianTrajectoryPosition-response"
  "da1eaa51dfbdd2438263c790b42891de")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlayCartesianTrajectoryPosition-response>)))
  "Returns full string definition for message of type '<PlayCartesianTrajectoryPosition-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlayCartesianTrajectoryPosition-response)))
  "Returns full string definition for message of type 'PlayCartesianTrajectoryPosition-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlayCartesianTrajectoryPosition-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlayCartesianTrajectoryPosition-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PlayCartesianTrajectoryPosition-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PlayCartesianTrajectoryPosition)))
  'PlayCartesianTrajectoryPosition-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PlayCartesianTrajectoryPosition)))
  'PlayCartesianTrajectoryPosition-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlayCartesianTrajectoryPosition)))
  "Returns string type for a service object of type '<PlayCartesianTrajectoryPosition>"
  "kortex_driver/PlayCartesianTrajectoryPosition")
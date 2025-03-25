; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude TakeSnapshot-request.msg.html

(cl:defclass <TakeSnapshot-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Snapshot
    :initform (cl:make-instance 'kortex_driver-msg:Snapshot)))
)

(cl:defclass TakeSnapshot-request (<TakeSnapshot-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TakeSnapshot-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TakeSnapshot-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<TakeSnapshot-request> is deprecated: use kortex_driver-srv:TakeSnapshot-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <TakeSnapshot-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TakeSnapshot-request>) ostream)
  "Serializes a message object of type '<TakeSnapshot-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TakeSnapshot-request>) istream)
  "Deserializes a message object of type '<TakeSnapshot-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TakeSnapshot-request>)))
  "Returns string type for a service object of type '<TakeSnapshot-request>"
  "kortex_driver/TakeSnapshotRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TakeSnapshot-request)))
  "Returns string type for a service object of type 'TakeSnapshot-request"
  "kortex_driver/TakeSnapshotRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TakeSnapshot-request>)))
  "Returns md5sum for a message object of type '<TakeSnapshot-request>"
  "4fa3de1c55135bce6fbd25c77b5e21af")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TakeSnapshot-request)))
  "Returns md5sum for a message object of type 'TakeSnapshot-request"
  "4fa3de1c55135bce6fbd25c77b5e21af")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TakeSnapshot-request>)))
  "Returns full string definition for message of type '<TakeSnapshot-request>"
  (cl:format cl:nil "Snapshot input~%~%================================================================================~%MSG: kortex_driver/Snapshot~%~%uint32 snapshot_type~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TakeSnapshot-request)))
  "Returns full string definition for message of type 'TakeSnapshot-request"
  (cl:format cl:nil "Snapshot input~%~%================================================================================~%MSG: kortex_driver/Snapshot~%~%uint32 snapshot_type~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TakeSnapshot-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TakeSnapshot-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TakeSnapshot-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude TakeSnapshot-response.msg.html

(cl:defclass <TakeSnapshot-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass TakeSnapshot-response (<TakeSnapshot-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TakeSnapshot-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TakeSnapshot-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<TakeSnapshot-response> is deprecated: use kortex_driver-srv:TakeSnapshot-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <TakeSnapshot-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TakeSnapshot-response>) ostream)
  "Serializes a message object of type '<TakeSnapshot-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TakeSnapshot-response>) istream)
  "Deserializes a message object of type '<TakeSnapshot-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TakeSnapshot-response>)))
  "Returns string type for a service object of type '<TakeSnapshot-response>"
  "kortex_driver/TakeSnapshotResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TakeSnapshot-response)))
  "Returns string type for a service object of type 'TakeSnapshot-response"
  "kortex_driver/TakeSnapshotResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TakeSnapshot-response>)))
  "Returns md5sum for a message object of type '<TakeSnapshot-response>"
  "4fa3de1c55135bce6fbd25c77b5e21af")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TakeSnapshot-response)))
  "Returns md5sum for a message object of type 'TakeSnapshot-response"
  "4fa3de1c55135bce6fbd25c77b5e21af")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TakeSnapshot-response>)))
  "Returns full string definition for message of type '<TakeSnapshot-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TakeSnapshot-response)))
  "Returns full string definition for message of type 'TakeSnapshot-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TakeSnapshot-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TakeSnapshot-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TakeSnapshot-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TakeSnapshot)))
  'TakeSnapshot-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TakeSnapshot)))
  'TakeSnapshot-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TakeSnapshot)))
  "Returns string type for a service object of type '<TakeSnapshot>"
  "kortex_driver/TakeSnapshot")
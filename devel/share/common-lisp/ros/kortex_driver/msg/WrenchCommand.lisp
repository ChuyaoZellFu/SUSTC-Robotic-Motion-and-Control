; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude WrenchCommand.msg.html

(cl:defclass <WrenchCommand> (roslisp-msg-protocol:ros-message)
  ((reference_frame
    :reader reference_frame
    :initarg :reference_frame
    :type cl:integer
    :initform 0)
   (mode
    :reader mode
    :initarg :mode
    :type cl:integer
    :initform 0)
   (wrench
    :reader wrench
    :initarg :wrench
    :type kortex_driver-msg:Wrench
    :initform (cl:make-instance 'kortex_driver-msg:Wrench))
   (duration
    :reader duration
    :initarg :duration
    :type cl:integer
    :initform 0))
)

(cl:defclass WrenchCommand (<WrenchCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WrenchCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WrenchCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<WrenchCommand> is deprecated: use kortex_driver-msg:WrenchCommand instead.")))

(cl:ensure-generic-function 'reference_frame-val :lambda-list '(m))
(cl:defmethod reference_frame-val ((m <WrenchCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:reference_frame-val is deprecated.  Use kortex_driver-msg:reference_frame instead.")
  (reference_frame m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <WrenchCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:mode-val is deprecated.  Use kortex_driver-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'wrench-val :lambda-list '(m))
(cl:defmethod wrench-val ((m <WrenchCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:wrench-val is deprecated.  Use kortex_driver-msg:wrench instead.")
  (wrench m))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <WrenchCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:duration-val is deprecated.  Use kortex_driver-msg:duration instead.")
  (duration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WrenchCommand>) ostream)
  "Serializes a message object of type '<WrenchCommand>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'reference_frame)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'reference_frame)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'reference_frame)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'reference_frame)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'mode)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'wrench) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'duration)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'duration)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'duration)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'duration)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WrenchCommand>) istream)
  "Deserializes a message object of type '<WrenchCommand>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'reference_frame)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'reference_frame)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'reference_frame)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'reference_frame)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'mode)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'wrench) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'duration)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'duration)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'duration)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'duration)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WrenchCommand>)))
  "Returns string type for a message object of type '<WrenchCommand>"
  "kortex_driver/WrenchCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WrenchCommand)))
  "Returns string type for a message object of type 'WrenchCommand"
  "kortex_driver/WrenchCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WrenchCommand>)))
  "Returns md5sum for a message object of type '<WrenchCommand>"
  "76c8b85ff99edd42232a407e2d24c818")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WrenchCommand)))
  "Returns md5sum for a message object of type 'WrenchCommand"
  "76c8b85ff99edd42232a407e2d24c818")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WrenchCommand>)))
  "Returns full string definition for message of type '<WrenchCommand>"
  (cl:format cl:nil "~%uint32 reference_frame~%uint32 mode~%Wrench wrench~%uint32 duration~%================================================================================~%MSG: kortex_driver/Wrench~%~%float32 force_x~%float32 force_y~%float32 force_z~%float32 torque_x~%float32 torque_y~%float32 torque_z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WrenchCommand)))
  "Returns full string definition for message of type 'WrenchCommand"
  (cl:format cl:nil "~%uint32 reference_frame~%uint32 mode~%Wrench wrench~%uint32 duration~%================================================================================~%MSG: kortex_driver/Wrench~%~%float32 force_x~%float32 force_y~%float32 force_z~%float32 torque_x~%float32 torque_y~%float32 torque_z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WrenchCommand>))
  (cl:+ 0
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'wrench))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WrenchCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'WrenchCommand
    (cl:cons ':reference_frame (reference_frame msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':wrench (wrench msg))
    (cl:cons ':duration (duration msg))
))

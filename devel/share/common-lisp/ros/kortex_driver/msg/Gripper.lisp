; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude Gripper.msg.html

(cl:defclass <Gripper> (roslisp-msg-protocol:ros-message)
  ((finger
    :reader finger
    :initarg :finger
    :type (cl:vector kortex_driver-msg:Finger)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:Finger :initial-element (cl:make-instance 'kortex_driver-msg:Finger))))
)

(cl:defclass Gripper (<Gripper>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Gripper>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Gripper)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<Gripper> is deprecated: use kortex_driver-msg:Gripper instead.")))

(cl:ensure-generic-function 'finger-val :lambda-list '(m))
(cl:defmethod finger-val ((m <Gripper>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:finger-val is deprecated.  Use kortex_driver-msg:finger instead.")
  (finger m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Gripper>) ostream)
  "Serializes a message object of type '<Gripper>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'finger))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'finger))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Gripper>) istream)
  "Deserializes a message object of type '<Gripper>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'finger) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'finger)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:Finger))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Gripper>)))
  "Returns string type for a message object of type '<Gripper>"
  "kortex_driver/Gripper")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Gripper)))
  "Returns string type for a message object of type 'Gripper"
  "kortex_driver/Gripper")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Gripper>)))
  "Returns md5sum for a message object of type '<Gripper>"
  "d6de7f2f0e25d1f1553a193c7d388b15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Gripper)))
  "Returns md5sum for a message object of type 'Gripper"
  "d6de7f2f0e25d1f1553a193c7d388b15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Gripper>)))
  "Returns full string definition for message of type '<Gripper>"
  (cl:format cl:nil "~%Finger[] finger~%================================================================================~%MSG: kortex_driver/Finger~%~%uint32 finger_identifier~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Gripper)))
  "Returns full string definition for message of type 'Gripper"
  (cl:format cl:nil "~%Finger[] finger~%================================================================================~%MSG: kortex_driver/Finger~%~%uint32 finger_identifier~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Gripper>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'finger) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Gripper>))
  "Converts a ROS message object to a list"
  (cl:list 'Gripper
    (cl:cons ':finger (finger msg))
))

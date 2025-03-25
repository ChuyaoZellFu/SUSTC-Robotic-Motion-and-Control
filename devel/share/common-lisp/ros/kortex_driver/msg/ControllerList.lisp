; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ControllerList.msg.html

(cl:defclass <ControllerList> (roslisp-msg-protocol:ros-message)
  ((handles
    :reader handles
    :initarg :handles
    :type (cl:vector kortex_driver-msg:ControllerHandle)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:ControllerHandle :initial-element (cl:make-instance 'kortex_driver-msg:ControllerHandle))))
)

(cl:defclass ControllerList (<ControllerList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControllerList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControllerList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ControllerList> is deprecated: use kortex_driver-msg:ControllerList instead.")))

(cl:ensure-generic-function 'handles-val :lambda-list '(m))
(cl:defmethod handles-val ((m <ControllerList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:handles-val is deprecated.  Use kortex_driver-msg:handles instead.")
  (handles m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControllerList>) ostream)
  "Serializes a message object of type '<ControllerList>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'handles))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'handles))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControllerList>) istream)
  "Deserializes a message object of type '<ControllerList>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'handles) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'handles)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:ControllerHandle))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControllerList>)))
  "Returns string type for a message object of type '<ControllerList>"
  "kortex_driver/ControllerList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerList)))
  "Returns string type for a message object of type 'ControllerList"
  "kortex_driver/ControllerList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControllerList>)))
  "Returns md5sum for a message object of type '<ControllerList>"
  "952a49d983a336790fc984bb01df940e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControllerList)))
  "Returns md5sum for a message object of type 'ControllerList"
  "952a49d983a336790fc984bb01df940e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControllerList>)))
  "Returns full string definition for message of type '<ControllerList>"
  (cl:format cl:nil "~%ControllerHandle[] handles~%================================================================================~%MSG: kortex_driver/ControllerHandle~%~%uint32 type~%uint32 controller_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControllerList)))
  "Returns full string definition for message of type 'ControllerList"
  (cl:format cl:nil "~%ControllerHandle[] handles~%================================================================================~%MSG: kortex_driver/ControllerHandle~%~%uint32 type~%uint32 controller_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControllerList>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'handles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControllerList>))
  "Converts a ROS message object to a list"
  (cl:list 'ControllerList
    (cl:cons ':handles (handles msg))
))

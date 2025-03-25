; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude CustomDataSelection.msg.html

(cl:defclass <CustomDataSelection> (roslisp-msg-protocol:ros-message)
  ((channel
    :reader channel
    :initarg :channel
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass CustomDataSelection (<CustomDataSelection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CustomDataSelection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CustomDataSelection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<CustomDataSelection> is deprecated: use kortex_driver-msg:CustomDataSelection instead.")))

(cl:ensure-generic-function 'channel-val :lambda-list '(m))
(cl:defmethod channel-val ((m <CustomDataSelection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:channel-val is deprecated.  Use kortex_driver-msg:channel instead.")
  (channel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CustomDataSelection>) ostream)
  "Serializes a message object of type '<CustomDataSelection>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'channel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'channel))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CustomDataSelection>) istream)
  "Deserializes a message object of type '<CustomDataSelection>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'channel) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'channel)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CustomDataSelection>)))
  "Returns string type for a message object of type '<CustomDataSelection>"
  "kortex_driver/CustomDataSelection")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CustomDataSelection)))
  "Returns string type for a message object of type 'CustomDataSelection"
  "kortex_driver/CustomDataSelection")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CustomDataSelection>)))
  "Returns md5sum for a message object of type '<CustomDataSelection>"
  "308bb0056e133aee362395c22802fb41")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CustomDataSelection)))
  "Returns md5sum for a message object of type 'CustomDataSelection"
  "308bb0056e133aee362395c22802fb41")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CustomDataSelection>)))
  "Returns full string definition for message of type '<CustomDataSelection>"
  (cl:format cl:nil "~%uint32[] channel~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CustomDataSelection)))
  "Returns full string definition for message of type 'CustomDataSelection"
  (cl:format cl:nil "~%uint32[] channel~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CustomDataSelection>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'channel) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CustomDataSelection>))
  "Converts a ROS message object to a list"
  (cl:list 'CustomDataSelection
    (cl:cons ':channel (channel msg))
))

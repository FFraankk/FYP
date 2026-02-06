现在hdl是用serive来开启定位的，以后可以用给/initialpose发消息来看哪些点比较近来完成

先运行 rosrun tf tf_monitor map camera_init。

看看 Broadcaster 那一栏。如果同时出现了 hdl_localization 和 hdl_governor_node，那就证实了冲突。
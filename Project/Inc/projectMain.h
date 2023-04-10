#ifndef PROJECTMAIN_H
#define PROJECTMAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

    void projectMain();
    void transfer_complete_callback(void);
    void transfer_error_callback(void);
    void receive_complete_callback(void);
    void receive_error_callback(void);
    void Error_Callback(void);

#ifdef __cplusplus
}
#endif

#endif /* PROJECTMAIN_H */
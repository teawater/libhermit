struct ibv_cq * ibv_cq_ex_to_cq(struct ibv_cq_ex * cq);
int ibv_start_poll(struct ibv_cq_ex * cq, struct ibv_poll_cq_attr * attr);
int ibv_next_poll(struct ibv_cq_ex * cq);
void ibv_end_poll(struct ibv_cq_ex * cq);
uint64_t ibv_wc_read_completion_ts(struct ibv_cq_ex * cq);
struct ibv_cq_ex * ibv_create_cq_ex(struct ibv_context * context, struct ibv_cq_init_attr_ex * cq_attr);
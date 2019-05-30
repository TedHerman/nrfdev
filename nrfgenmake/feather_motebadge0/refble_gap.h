typedef struct
{
  uint8_t addr_id_peer : 1;       /**< Only valid for peer addresses.
                                       This bit is set by the SoftDevice to indicate whether the address has been resolved from
                                       a Resolvable Private Address (when the peer is using privacy).
                                       If set to 1, @ref addr and @ref addr_type refer to the identity address of the resolved address.

                                       This bit is ignored when a variable of type @ref ble_gap_addr_t is used as input to API functions. */
  uint8_t addr_type    : 7;       /**< See @ref BLE_GAP_ADDR_TYPES. */
  uint8_t addr[BLE_GAP_ADDR_LEN]; /**< 48-bit address, LSB format.
                                       @ref addr is not used if @ref addr_type is @ref BLE_GAP_ADDR_TYPE_ANONYMOUS. */
} ble_gap_addr_t;


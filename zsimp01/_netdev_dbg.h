
struct e1000_reg_info {
	u32 ofs;
	char *name;
};

static const struct e1000_reg_info e1000_reg_info_tbl[] = {
	/* General Registers */
	{E1000_CTRL, "CTRL"},
	{E1000_STATUS, "STATUS"},
	{E1000_CTRL_EXT, "CTRL_EXT"},

	/* Interrupt Registers */
	{E1000_ICR, "ICR"},

	/* Rx Registers */
	{E1000_RCTL, "RCTL"},
	{E1000_RDLEN(0), "RDLEN"},
	{E1000_RDH(0), "RDH"},
	{E1000_RDT(0), "RDT"},
	{E1000_RDTR, "RDTR"},
	{E1000_RXDCTL(0), "RXDCTL"},
	{E1000_ERT, "ERT"},
	{E1000_RDBAL(0), "RDBAL"},
	{E1000_RDBAH(0), "RDBAH"},
	{E1000_RDFH, "RDFH"},
	{E1000_RDFT, "RDFT"},
	{E1000_RDFHS, "RDFHS"},
	{E1000_RDFTS, "RDFTS"},
	{E1000_RDFPC, "RDFPC"},

	/* Tx Registers */
	{E1000_TCTL, "TCTL"},
	{E1000_TDBAL(0), "TDBAL"},
	{E1000_TDBAH(0), "TDBAH"},
	{E1000_TDLEN(0), "TDLEN"},
	{E1000_TDH(0), "TDH"},
	{E1000_TDT(0), "TDT"},
	{E1000_TIDV, "TIDV"},
	{E1000_TXDCTL(0), "TXDCTL"},
	{E1000_TADV, "TADV"},
	{E1000_TARC(0), "TARC"},
	{E1000_TDFH, "TDFH"},
	{E1000_TDFT, "TDFT"},
	{E1000_TDFHS, "TDFHS"},
	{E1000_TDFTS, "TDFTS"},
	{E1000_TDFPC, "TDFPC"},

	/* List Terminator */
	{0, NULL}
};

/**
 * e1000_regdump - register printout routine
 * @hw: pointer to the HW structure
 * @reginfo: pointer to the register info table
 **/
static void e1000_regdump(struct e1000_hw *hw, struct e1000_reg_info *reginfo)
{
	int n = 0;
	char rname[16];
	u32 regs[8];

	switch (reginfo->ofs) {
	case E1000_RXDCTL(0):
		for (n = 0; n < 2; n++)
			regs[n] = __er32(hw, E1000_RXDCTL(n));
		break;
	case E1000_TXDCTL(0):
		for (n = 0; n < 2; n++)
			regs[n] = __er32(hw, E1000_TXDCTL(n));
		break;
	case E1000_TARC(0):
		for (n = 0; n < 2; n++)
			regs[n] = __er32(hw, E1000_TARC(n));
		break;
	default:
		pr_info("%-15s %08x\n",
			reginfo->name, __er32(hw, reginfo->ofs));
		return;
	}

	snprintf(rname, 16, "%s%s", reginfo->name, "[0-1]");
	pr_info("%-15s %08x %08x\n", rname, regs[0], regs[1]);
}

static void e1000e_dump_ps_pages(struct e1000_adapter *adapter,
				 struct e1000_buffer *bi)
{
	int i;
	struct e1000_ps_page *ps_page;

	for (i = 0; i < adapter->rx_ps_pages; i++) {
		ps_page = &bi->ps_pages[i];

		if (ps_page->page) {
			pr_info("packet dump for ps_page %d:\n", i);
			print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS,
				       16, 1, page_address(ps_page->page),
				       PAGE_SIZE, true);
		}
	}
}

/**
 * e1000e_dump - Print registers, Tx-ring and Rx-ring
 * @adapter: board private structure
 **/
static void e1000e_dump(struct e1000_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;
	struct e1000_hw *hw = &adapter->hw;
	struct e1000_reg_info *reginfo;
	struct e1000_ring *tx_ring = adapter->tx_ring;
	struct e1000_tx_desc *tx_desc;
	struct my_u0 {
		__le64 a;
		__le64 b;
	} *u0;
	struct e1000_buffer *buffer_info;
	struct e1000_ring *rx_ring = adapter->rx_ring;
	union e1000_rx_desc_packet_split *rx_desc_ps;
	union e1000_rx_desc_extended *rx_desc;
	struct my_u1 {
		__le64 a;
		__le64 b;
		__le64 c;
		__le64 d;
	} *u1;
	u32 staterr;
	int i = 0;

	if (!netif_msg_hw(adapter))
		return;

	/* Print netdevice Info */
	if (netdev) {
		dev_info(&adapter->pdev->dev, "Net device Info\n");
		pr_info("Device Name     state            trans_start      last_rx\n");
		pr_info("%-15s %016lX %016lX %016lX\n", netdev->name,
			netdev->state, netdev->trans_start, netdev->last_rx);
	}

	/* Print Registers */
	dev_info(&adapter->pdev->dev, "Register Dump\n");
	pr_info(" Register Name   Value\n");
	for (reginfo = (struct e1000_reg_info *)e1000_reg_info_tbl;
	     reginfo->name; reginfo++) {
		e1000_regdump(hw, reginfo);
	}

	/* Print Tx Ring Summary */
	if (!netdev || !netif_running(netdev))
		return;

	dev_info(&adapter->pdev->dev, "Tx Ring Summary\n");
	pr_info("Queue [NTU] [NTC] [bi(ntc)->dma  ] leng ntw timestamp\n");
	buffer_info = &tx_ring->buffer_info[tx_ring->next_to_clean];
	pr_info(" %5d %5X %5X %016llX %04X %3X %016llX\n",
		0, tx_ring->next_to_use, tx_ring->next_to_clean,
		(unsigned long long)buffer_info->dma,
		buffer_info->length,
		buffer_info->next_to_watch,
		(unsigned long long)buffer_info->time_stamp);

	/* Print Tx Ring */
	if (!netif_msg_tx_done(adapter))
		goto rx_ring_summary;

	dev_info(&adapter->pdev->dev, "Tx Ring Dump\n");

	/* Transmit Descriptor Formats - DEXT[29] is 0 (Legacy) or 1 (Extended)
	 *
	 * Legacy Transmit Descriptor
	 *   +--------------------------------------------------------------+
	 * 0 |         Buffer Address [63:0] (Reserved on Write Back)       |
	 *   +--------------------------------------------------------------+
	 * 8 | Special  |    CSS     | Status |  CMD    |  CSO   |  Length  |
	 *   +--------------------------------------------------------------+
	 *   63       48 47        36 35    32 31     24 23    16 15        0
	 *
	 * Extended Context Descriptor (DTYP=0x0) for TSO or checksum offload
	 *   63      48 47    40 39       32 31             16 15    8 7      0
	 *   +----------------------------------------------------------------+
	 * 0 |  TUCSE  | TUCS0  |   TUCSS   |     IPCSE       | IPCS0 | IPCSS |
	 *   +----------------------------------------------------------------+
	 * 8 |   MSS   | HDRLEN | RSV | STA | TUCMD | DTYP |      PAYLEN      |
	 *   +----------------------------------------------------------------+
	 *   63      48 47    40 39 36 35 32 31   24 23  20 19                0
	 *
	 * Extended Data Descriptor (DTYP=0x1)
	 *   +----------------------------------------------------------------+
	 * 0 |                     Buffer Address [63:0]                      |
	 *   +----------------------------------------------------------------+
	 * 8 | VLAN tag |  POPTS  | Rsvd | Status | Command | DTYP |  DTALEN  |
	 *   +----------------------------------------------------------------+
	 *   63       48 47     40 39  36 35    32 31     24 23  20 19        0
	 */
	pr_info("Tl[desc]     [address 63:0  ] [SpeCssSCmCsLen] [bi->dma       ] leng  ntw timestamp        bi->skb <-- Legacy format\n");
	pr_info("Tc[desc]     [Ce CoCsIpceCoS] [MssHlRSCm0Plen] [bi->dma       ] leng  ntw timestamp        bi->skb <-- Ext Context format\n");
	pr_info("Td[desc]     [address 63:0  ] [VlaPoRSCm1Dlen] [bi->dma       ] leng  ntw timestamp        bi->skb <-- Ext Data format\n");
	for (i = 0; tx_ring->desc && (i < tx_ring->count); i++) {
		const char *next_desc;
		tx_desc = E1000_TX_DESC(*tx_ring, i);
		buffer_info = &tx_ring->buffer_info[i];
		u0 = (struct my_u0 *)tx_desc;
		if (i == tx_ring->next_to_use && i == tx_ring->next_to_clean)
			next_desc = " NTC/U";
		else if (i == tx_ring->next_to_use)
			next_desc = " NTU";
		else if (i == tx_ring->next_to_clean)
			next_desc = " NTC";
		else
			next_desc = "";
		pr_info("T%c[0x%03X]    %016llX %016llX %016llX %04X  %3X %016llX %p%s\n",
			(!(le64_to_cpu(u0->b) & (1 << 29)) ? 'l' :
			 ((le64_to_cpu(u0->b) & (1 << 20)) ? 'd' : 'c')),
			i,
			(unsigned long long)le64_to_cpu(u0->a),
			(unsigned long long)le64_to_cpu(u0->b),
			(unsigned long long)buffer_info->dma,
			buffer_info->length, buffer_info->next_to_watch,
			(unsigned long long)buffer_info->time_stamp,
			buffer_info->skb, next_desc);

		if (netif_msg_pktdata(adapter) && buffer_info->skb)
			print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS,
				       16, 1, buffer_info->skb->data,
				       buffer_info->skb->len, true);
	}

	/* Print Rx Ring Summary */
rx_ring_summary:
	dev_info(&adapter->pdev->dev, "Rx Ring Summary\n");
	pr_info("Queue [NTU] [NTC]\n");
	pr_info(" %5d %5X %5X\n",
		0, rx_ring->next_to_use, rx_ring->next_to_clean);

	/* Print Rx Ring */
	if (!netif_msg_rx_status(adapter))
		return;

	dev_info(&adapter->pdev->dev, "Rx Ring Dump\n");
	switch (adapter->rx_ps_pages) {
	case 1:
	case 2:
	case 3:
		/* [Extended] Packet Split Receive Descriptor Format
		 *
		 *    +-----------------------------------------------------+
		 *  0 |                Buffer Address 0 [63:0]              |
		 *    +-----------------------------------------------------+
		 *  8 |                Buffer Address 1 [63:0]              |
		 *    +-----------------------------------------------------+
		 * 16 |                Buffer Address 2 [63:0]              |
		 *    +-----------------------------------------------------+
		 * 24 |                Buffer Address 3 [63:0]              |
		 *    +-----------------------------------------------------+
		 */
		pr_info("R  [desc]      [buffer 0 63:0 ] [buffer 1 63:0 ] [buffer 2 63:0 ] [buffer 3 63:0 ] [bi->dma       ] [bi->skb] <-- Ext Pkt Split format\n");
		/* [Extended] Receive Descriptor (Write-Back) Format
		 *
		 *   63       48 47    32 31     13 12    8 7    4 3        0
		 *   +------------------------------------------------------+
		 * 0 | Packet   | IP     |  Rsvd   | MRQ   | Rsvd | MRQ RSS |
		 *   | Checksum | Ident  |         | Queue |      |  Type   |
		 *   +------------------------------------------------------+
		 * 8 | VLAN Tag | Length | Extended Error | Extended Status |
		 *   +------------------------------------------------------+
		 *   63       48 47    32 31            20 19               0
		 */
		pr_info("RWB[desc]      [ck ipid mrqhsh] [vl   l0 ee  es] [ l3  l2  l1 hs] [reserved      ] ---------------- [bi->skb] <-- Ext Rx Write-Back format\n");
		for (i = 0; i < rx_ring->count; i++) {
			const char *next_desc;
			buffer_info = &rx_ring->buffer_info[i];
			rx_desc_ps = E1000_RX_DESC_PS(*rx_ring, i);
			u1 = (struct my_u1 *)rx_desc_ps;
			staterr =
			    le32_to_cpu(rx_desc_ps->wb.middle.status_error);

			if (i == rx_ring->next_to_use)
				next_desc = " NTU";
			else if (i == rx_ring->next_to_clean)
				next_desc = " NTC";
			else
				next_desc = "";

			if (staterr & E1000_RXD_STAT_DD) {
				/* Descriptor Done */
				pr_info("%s[0x%03X]     %016llX %016llX %016llX %016llX ---------------- %p%s\n",
					"RWB", i,
					(unsigned long long)le64_to_cpu(u1->a),
					(unsigned long long)le64_to_cpu(u1->b),
					(unsigned long long)le64_to_cpu(u1->c),
					(unsigned long long)le64_to_cpu(u1->d),
					buffer_info->skb, next_desc);
			} else {
				pr_info("%s[0x%03X]     %016llX %016llX %016llX %016llX %016llX %p%s\n",
					"R  ", i,
					(unsigned long long)le64_to_cpu(u1->a),
					(unsigned long long)le64_to_cpu(u1->b),
					(unsigned long long)le64_to_cpu(u1->c),
					(unsigned long long)le64_to_cpu(u1->d),
					(unsigned long long)buffer_info->dma,
					buffer_info->skb, next_desc);

				if (netif_msg_pktdata(adapter))
					e1000e_dump_ps_pages(adapter,
							     buffer_info);
			}
		}
		break;
	default:
	case 0:
		/* Extended Receive Descriptor (Read) Format
		 *
		 *   +-----------------------------------------------------+
		 * 0 |                Buffer Address [63:0]                |
		 *   +-----------------------------------------------------+
		 * 8 |                      Reserved                       |
		 *   +-----------------------------------------------------+
		 */
		pr_info("R  [desc]      [buf addr 63:0 ] [reserved 63:0 ] [bi->dma       ] [bi->skb] <-- Ext (Read) format\n");
		/* Extended Receive Descriptor (Write-Back) Format
		 *
		 *   63       48 47    32 31    24 23            4 3        0
		 *   +------------------------------------------------------+
		 *   |     RSS Hash      |        |               |         |
		 * 0 +-------------------+  Rsvd  |   Reserved    | MRQ RSS |
		 *   | Packet   | IP     |        |               |  Type   |
		 *   | Checksum | Ident  |        |               |         |
		 *   +------------------------------------------------------+
		 * 8 | VLAN Tag | Length | Extended Error | Extended Status |
		 *   +------------------------------------------------------+
		 *   63       48 47    32 31            20 19               0
		 */
		pr_info("RWB[desc]      [cs ipid    mrq] [vt   ln xe  xs] [bi->skb] <-- Ext (Write-Back) format\n");

		for (i = 0; i < rx_ring->count; i++) {
			const char *next_desc;

			buffer_info = &rx_ring->buffer_info[i];
			rx_desc = E1000_RX_DESC_EXT(*rx_ring, i);
			u1 = (struct my_u1 *)rx_desc;
			staterr = le32_to_cpu(rx_desc->wb.upper.status_error);

			if (i == rx_ring->next_to_use)
				next_desc = " NTU";
			else if (i == rx_ring->next_to_clean)
				next_desc = " NTC";
			else
				next_desc = "";

			if (staterr & E1000_RXD_STAT_DD) {
				/* Descriptor Done */
				pr_info("%s[0x%03X]     %016llX %016llX ---------------- %p%s\n",
					"RWB", i,
					(unsigned long long)le64_to_cpu(u1->a),
					(unsigned long long)le64_to_cpu(u1->b),
					buffer_info->skb, next_desc);
			} else {
				pr_info("%s[0x%03X]     %016llX %016llX %016llX %p%s\n",
					"R  ", i,
					(unsigned long long)le64_to_cpu(u1->a),
					(unsigned long long)le64_to_cpu(u1->b),
					(unsigned long long)buffer_info->dma,
					buffer_info->skb, next_desc);

				if (netif_msg_pktdata(adapter) &&
				    buffer_info->skb)
					print_hex_dump(KERN_INFO, "",
						       DUMP_PREFIX_ADDRESS, 16,
						       1,
						       buffer_info->skb->data,
						       adapter->rx_buffer_len,
						       true);
			}
		}
	}
}

/**
 * e1000_desc_unused - calculate if we have unused descriptors
 **/
static int e1000_desc_unused(struct e1000_ring *ring)
{
	if (ring->next_to_clean > ring->next_to_use)
		return ring->next_to_clean - ring->next_to_use - 1;

	return ring->count + ring->next_to_clean - ring->next_to_use - 1;
}

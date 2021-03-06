
/**
 * e1000_intr_msi - Interrupt Handler
 * @irq: interrupt number
 * @data: pointer to a network interface device structure
 **/
static irqreturn_t e1000_intr_msi(int __always_unused irq, void *data)
{
	struct net_device *netdev = data;
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	u32 icr = er32(ICR);

	/* read ICR disables interrupts using IAM */
	if (icr & E1000_ICR_LSC) {
		hw->mac.get_link_status = true;
		/* ICH8 workaround-- Call gig speed drop workaround on cable
		 * disconnect (LSC) before accessing any PHY registers
		 */
		if ((adapter->flags & FLAG_LSC_GIG_SPEED_DROP) &&
		    (!(er32(STATUS) & E1000_STATUS_LU)))
			schedule_work(&adapter->downshift_task);

		/* 80003ES2LAN workaround-- For packet buffer work-around on
		 * link down event; disable receives here in the ISR and reset
		 * adapter in watchdog
		 */
		if (netif_carrier_ok(netdev) &&
		    adapter->flags & FLAG_RX_NEEDS_RESTART) {
			/* disable receives */
			u32 rctl = er32(RCTL);
			ew32(RCTL, rctl & ~E1000_RCTL_EN);
			adapter->flags |= FLAG_RESTART_NOW;
		}
		/* guard against interrupt when we're going down */
		if (!test_bit(__E1000_DOWN, &adapter->state))
			mod_timer(&adapter->watchdog_timer, jiffies + 1);
	}

	/* Reset on uncorrectable ECC error */
	if ((icr & E1000_ICR_ECCER) && (hw->mac.type == e1000_pch_lpt)) {
		u32 pbeccsts = er32(PBECCSTS);

		adapter->corr_errors +=
		    pbeccsts & E1000_PBECCSTS_CORR_ERR_CNT_MASK;
		adapter->uncorr_errors +=
		    (pbeccsts & E1000_PBECCSTS_UNCORR_ERR_CNT_MASK) >>
		    E1000_PBECCSTS_UNCORR_ERR_CNT_SHIFT;

		/* Do the reset outside of interrupt context */
		schedule_work(&adapter->reset_task);

		/* return immediately since reset is imminent */
		return IRQ_HANDLED;
	}

	if (napi_schedule_prep(&adapter->napi)) {
		adapter->total_tx_bytes = 0;
		adapter->total_tx_packets = 0;
		adapter->total_rx_bytes = 0;
		adapter->total_rx_packets = 0;
		__napi_schedule(&adapter->napi);
	}

	return IRQ_HANDLED;
}

/**
 * e1000_intr_msi_test - Interrupt Handler
 * @irq: interrupt number
 * @data: pointer to a network interface device structure
 **/
static irqreturn_t e1000_intr_msi_test(int __always_unused irq, void *data)
{
	struct net_device *netdev = data;
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	u32 icr = er32(ICR);

	e_dbg("icr is %08X\n", icr);
	if (icr & E1000_ICR_RXSEQ) {
		adapter->flags &= ~FLAG_MSI_TEST_FAILED;
		/* Force memory writes to complete before acknowledging the
		 * interrupt is handled.
		 */
		wmb();
	}

	return IRQ_HANDLED;
}

/**
 * e1000_test_msi_interrupt - Returns 0 for successful test
 * @adapter: board private struct
 *
 * code flow taken from tg3.c
 **/
static int e1000_test_msi_interrupt(struct e1000_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;
	struct e1000_hw *hw = &adapter->hw;
	int err;

	/* poll_enable hasn't been called yet, so don't need disable */
	/* clear any pending events */
	er32(ICR);

	/* free the real vector and request a test handler */
	e1000_free_irq(adapter);
	e1000e_reset_interrupt_capability(adapter);

	/* Assume that the test fails, if it succeeds then the test
	 * MSI irq handler will unset this flag
	 */
	adapter->flags |= FLAG_MSI_TEST_FAILED;

	err = pci_enable_msi(adapter->pdev);
	if (err)
		goto msi_test_failed;

	err = request_irq(adapter->pdev->irq, e1000_intr_msi_test, 0,
			  netdev->name, netdev);
	if (err) {
		pci_disable_msi(adapter->pdev);
		goto msi_test_failed;
	}

	/* Force memory writes to complete before enabling and firing an
	 * interrupt.
	 */
	wmb();

	e1000_irq_enable(adapter);

	/* fire an unusual interrupt on the test handler */
	ew32(ICS, E1000_ICS_RXSEQ);
	e1e_flush();
	msleep(100);

	e1000_irq_disable(adapter);

	rmb();			/* read flags after interrupt has been fired */

		e_dbg("MSI interrupt test succeeded!\n");

	free_irq(adapter->pdev->irq, netdev);
	pci_disable_msi(adapter->pdev);

msi_test_failed:
	e1000e_set_interrupt_capability(adapter);
	return e1000_request_irq(adapter);
}

/**
 * e1000_test_msi - Returns 0 if MSI test succeeds or INTx mode is restored
 * @adapter: board private struct
 *
 * code flow taken from tg3.c, called with e1000 interrupts disabled.
 **/
static int e1000_test_msi(struct e1000_adapter *adapter)
{
	int err;
	u16 pci_cmd;

	if (!(adapter->flags & FLAG_MSI_ENABLED))
		return 0;

	/* disable SERR in case the MSI write causes a master abort */
	pci_read_config_word(adapter->pdev, PCI_COMMAND, &pci_cmd);
	if (pci_cmd & PCI_COMMAND_SERR)
		pci_write_config_word(adapter->pdev, PCI_COMMAND,
				      pci_cmd & ~PCI_COMMAND_SERR);

	err = e1000_test_msi_interrupt(adapter);

	/* re-enable SERR */
	if (pci_cmd & PCI_COMMAND_SERR) {
		pci_read_config_word(adapter->pdev, PCI_COMMAND, &pci_cmd);
		pci_cmd |= PCI_COMMAND_SERR;
		pci_write_config_word(adapter->pdev, PCI_COMMAND, pci_cmd);
	}

	return err;
}


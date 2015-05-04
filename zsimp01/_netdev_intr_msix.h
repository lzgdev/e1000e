
// _netdev_rxtx.h
static bool e1000_clean_tx_irq(struct e1000_ring *tx_ring);

static irqreturn_t e1000_msix_other(int __always_unused irq, void *data)
{
	struct net_device *netdev = data;
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	u32 icr = er32(ICR);

	if (!(icr & E1000_ICR_INT_ASSERTED)) {
		if (!test_bit(__E1000_DOWN, &adapter->state))
			ew32(IMS, E1000_IMS_OTHER);
		return IRQ_NONE;
	}

	if (icr & adapter->eiac_mask)
		ew32(ICS, (icr & adapter->eiac_mask));

	if (icr & E1000_ICR_OTHER) {
		if (!(icr & E1000_ICR_LSC))
			goto no_link_interrupt;
		hw->mac.get_link_status = true;
		/* guard against interrupt when we're going down */
		if (!test_bit(__E1000_DOWN, &adapter->state))
			mod_timer(&adapter->watchdog_timer, jiffies + 1);
	}

no_link_interrupt:
	if (!test_bit(__E1000_DOWN, &adapter->state))
		ew32(IMS, E1000_IMS_LSC | E1000_IMS_OTHER);

	return IRQ_HANDLED;
}

static irqreturn_t e1000_intr_msix_tx(int __always_unused irq, void *data)
{
	struct net_device *netdev = data;
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	struct e1000_ring *tx_ring = adapter->tx_ring;

	adapter->total_tx_bytes = 0;
	adapter->total_tx_packets = 0;

	if (!e1000_clean_tx_irq(tx_ring))
		/* Ring was not completely cleaned, so fire another interrupt */
		ew32(ICS, tx_ring->ims_val);

	return IRQ_HANDLED;
}

static irqreturn_t e1000_intr_msix_rx(int __always_unused irq, void *data)
{
	struct net_device *netdev = data;
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct e1000_ring *rx_ring = adapter->rx_ring;

	/* Write the ITR value calculated at the end of the
	 * previous interrupt.
	 */
	if (rx_ring->set_itr) {
		writel(1000000000 / (rx_ring->itr_val * 256),
		       rx_ring->itr_register);
		rx_ring->set_itr = 0;
	}

	if (napi_schedule_prep(&adapter->napi)) {
		adapter->total_rx_bytes = 0;
		adapter->total_rx_packets = 0;
		__napi_schedule(&adapter->napi);
	}
	return IRQ_HANDLED;
}

/**
 * e1000_configure_msix - Configure MSI-X hardware
 *
 * e1000_configure_msix sets up the hardware to properly
 * generate MSI-X interrupts.
 **/
static void e1000_configure_msix(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	struct e1000_ring *rx_ring = adapter->rx_ring;
	struct e1000_ring *tx_ring = adapter->tx_ring;
	int vector = 0;
	u32 ctrl_ext, ivar = 0;

	adapter->eiac_mask = 0;

	/* Configure Rx vector */
	rx_ring->ims_val = E1000_IMS_RXQ0;
	adapter->eiac_mask |= rx_ring->ims_val;
	if (rx_ring->itr_val)
		writel(1000000000 / (rx_ring->itr_val * 256),
		       rx_ring->itr_register);
	else
		writel(1, rx_ring->itr_register);
	ivar = E1000_IVAR_INT_ALLOC_VALID | vector;

	/* Configure Tx vector */
	tx_ring->ims_val = E1000_IMS_TXQ0;
	vector++;
	if (tx_ring->itr_val)
		writel(1000000000 / (tx_ring->itr_val * 256),
		       tx_ring->itr_register);
	else
		writel(1, tx_ring->itr_register);
	adapter->eiac_mask |= tx_ring->ims_val;
	ivar |= ((E1000_IVAR_INT_ALLOC_VALID | vector) << 8);

	/* set vector for Other Causes, e.g. link changes */
	vector++;
	ivar |= ((E1000_IVAR_INT_ALLOC_VALID | vector) << 16);
	if (rx_ring->itr_val)
		writel(1000000000 / (rx_ring->itr_val * 256),
		       hw->hw_addr + E1000_EITR_82574(vector));
	else
		writel(1, hw->hw_addr + E1000_EITR_82574(vector));

	/* Cause Tx interrupts on every write back */
	ivar |= (1 << 31);

	ew32(IVAR, ivar);

	/* enable MSI-X PBA support */
	ctrl_ext = er32(CTRL_EXT);
	ctrl_ext |= E1000_CTRL_EXT_PBA_CLR;

	/* Auto-Mask Other interrupts upon ICR read */
	ew32(IAM, ~E1000_EIAC_MASK_82574 | E1000_IMS_OTHER);
	ctrl_ext |= E1000_CTRL_EXT_EIAME;
	ew32(CTRL_EXT, ctrl_ext);
	e1e_flush();
}

/**
 * e1000_request_msix - Initialize MSI-X interrupts
 *
 * e1000_request_msix allocates MSI-X vectors and requests interrupts from the
 * kernel.
 **/
static int e1000_request_msix(struct e1000_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;
	int err = 0, vector = 0;

	if (strlen(netdev->name) < (IFNAMSIZ - 5))
		snprintf(adapter->rx_ring->name,
			 sizeof(adapter->rx_ring->name) - 1,
			 "%s-rx-0", netdev->name);
	else
		memcpy(adapter->rx_ring->name, netdev->name, IFNAMSIZ);
	err = request_irq(adapter->msix_entries[vector].vector,
			  e1000_intr_msix_rx, 0, adapter->rx_ring->name,
			  netdev);
	if (err)
		return err;
	adapter->rx_ring->itr_register = adapter->hw.hw_addr +
	    E1000_EITR_82574(vector);
	adapter->rx_ring->itr_val = adapter->itr;
	vector++;

	if (strlen(netdev->name) < (IFNAMSIZ - 5))
		snprintf(adapter->tx_ring->name,
			 sizeof(adapter->tx_ring->name) - 1,
			 "%s-tx-0", netdev->name);
	else
		memcpy(adapter->tx_ring->name, netdev->name, IFNAMSIZ);
	err = request_irq(adapter->msix_entries[vector].vector,
			  e1000_intr_msix_tx, 0, adapter->tx_ring->name,
			  netdev);
	if (err)
		return err;
	adapter->tx_ring->itr_register = adapter->hw.hw_addr +
	    E1000_EITR_82574(vector);
	adapter->tx_ring->itr_val = adapter->itr;
	vector++;

	err = request_irq(adapter->msix_entries[vector].vector,
			  e1000_msix_other, 0, netdev->name, netdev);
	if (err)
		return err;

	e1000_configure_msix(adapter);

	return 0;
}

#ifdef CONFIG_NET_POLL_CONTROLLER

static irqreturn_t e1000_intr_msix(int __always_unused irq, void *data)
{
	struct net_device *netdev = data;
	struct e1000_adapter *adapter = netdev_priv(netdev);

	if (adapter->msix_entries) {
		int vector, msix_irq;

		vector = 0;
		msix_irq = adapter->msix_entries[vector].vector;
		disable_irq(msix_irq);
		e1000_intr_msix_rx(msix_irq, netdev);
		enable_irq(msix_irq);

		vector++;
		msix_irq = adapter->msix_entries[vector].vector;
		disable_irq(msix_irq);
		e1000_intr_msix_tx(msix_irq, netdev);
		enable_irq(msix_irq);

		vector++;
		msix_irq = adapter->msix_entries[vector].vector;
		disable_irq(msix_irq);
		e1000_msix_other(msix_irq, netdev);
		enable_irq(msix_irq);
	}

	return IRQ_HANDLED;
}

/**
 * e1000_netpoll
 * @netdev: network interface device structure
 *
 * Polling 'interrupt' - used by things like netconsole to send skbs
 * without having to re-enable interrupts. It's not called while
 * the interrupt routine is executing.
 */
static void e1000_netpoll(struct net_device *netdev)
{
	struct e1000_adapter *adapter = netdev_priv(netdev);

	switch (adapter->int_mode) {
	case E1000E_INT_MODE_MSIX:
		e1000_intr_msix(adapter->pdev->irq, netdev);
		break;
	case E1000E_INT_MODE_MSI:
		disable_irq(adapter->pdev->irq);
		e1000_intr_msi(adapter->pdev->irq, netdev);
		enable_irq(adapter->pdev->irq);
		break;
	default:		/* E1000E_INT_MODE_LEGACY */
		disable_irq(adapter->pdev->irq);
		e1000_intr(adapter->pdev->irq, netdev);
		enable_irq(adapter->pdev->irq);
		break;
	}
}
#endif

